// SPDX-License-Identifier: BSD-2-Clause-Patent
// Polling USB 2 CDC ACM console for a DWC3 controller.
// No firmware services are called. The ADT supplies the controller aperture. Boards that need
// it can ask for the Qualcomm GCC/QUSB2/apps-SMMU preparation through the ADT; otherwise this
// consumes a controller that firmware already prepared.
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#ifndef UNIXNU_DWC3_HOST_TEST
#include <machine/machine_routines.h>
#include <pexpert/device_tree.h>
extern vm_offset_t ml_static_vtop(vm_offset_t);
#endif

#define EVENT_BYTES 4096u
#define RING_BYTES 32768u
#define EP_COUNT 6u
// Sirius/SDM710 board resources used only when /chosen/unixnu,qcom-usb-board-init is 1.
#define GCC_PHYS 0x00100000u
#define GCC_BYTES 0x001f0000u
#define GCC_USB30_PRIM_GDSCR 0xf004u
#define GCC_USB30_PRIM_MASTER 0xf00cu
#define GCC_USB30_PRIM_SLEEP 0xf010u
#define GCC_USB30_PRIM_MOCK_UTMI 0xf014u
#define GCC_CFG_NOC_USB3_PRIM 0x502cu
#define GCC_AGGRE_USB3_PRIM 0x8201cu
#define GCC_USB_PHY_CFG_AHB2PHY 0x6a004u
#define GCC_USB3_PRIM_CLKREF 0x8c008u
#define GCC_QUSB2PHY_PRIM_BCR 0x12000u
#define QUSB2_PHY_PHYS 0x088e2000u
#define QUSB2_PHY_BYTES 0x400u
#define QUSB2_PHY_POWERDOWN 0x210u
#define QUSB2_PHY_PLL_STATUS 0x1a0u
#define QUSB2_PHY_PLL_CORE_INPUT_OVERRIDE 0xa8u
#define QUSB2_PHY_PLL_LOCKED (1u << 5)
#define QUSB2_PHY_CORE_READY (1u << 0)
#define APPS_SMMU_PHYS 0x15000000u
#define APPS_SMMU_BYTES 0x000d0000u
#define USB_STREAM_ID 0x0740u
#define QSCRATCH_GENERAL_CFG 0xf8808u
#define QSCRATCH_HS_PHY_CTRL 0xf8810u
#define QSCRATCH_PIPE_UTMI_CLK_SEL (1u << 0)
#define QSCRATCH_PIPE3_PHYSTATUS_SW (1u << 3)
#define QSCRATCH_PIPE_UTMI_CLK_DIS (1u << 8)
#define QSCRATCH_UTMI_OTG_VBUS_VALID (1u << 20)
#define QSCRATCH_SW_SESSVLD_SEL (1u << 28)
#define GCTL 0xc110u
#define GSNPSID 0xc120u
#define GEVNTADRLO 0xc400u
#define GEVNTADRHI 0xc404u
#define GEVNTSIZ 0xc408u
#define GEVNTCOUNT 0xc40cu
#define DCFG 0xc700u
#define DCTL 0xc704u
#define DEVTEN 0xc708u
#define DSTS 0xc70cu
#define DALEPENA 0xc720u
#define DEP(n) (0xc800u + (n) * 16u)
// Board reset: PM660 PON_SOFT_RB_SPARE carries the bootloader reason, so a PSCI reset alone
// reboots the system. Writing mode-bootloader makes the next ABL stop in fastboot.
#define SPMI_CORE_PHYS 0x0c440000u
#define SPMI_CNFG_PHYS 0x0c40a000u
#define SPMI_CHNLS_PHYS 0x0c600000u
#define SPMI_OBS_PHYS 0x0e600000u
#define SPMI_WINDOW 0x2000u
#define SPMI_ARB_VERSION 0x0000u
#define SPMI_ARB_FEATURES 0x0004u
#define SPMI_APID_MAP_V5 0x0900u
#define SPMI_APID_MAP_V3 0x0800u
#define SPMI_REG_CMD0 0x00u
#define SPMI_REG_CONFIG 0x04u
#define SPMI_REG_STATUS 0x08u
#define SPMI_REG_WDATA 0x10u
#define SPMI_REG_RDATA 0x18u
#define SPMI_CMD_WRITE_LONG 0x00u
#define SPMI_CMD_READ_LONG 0x01u
#define PON_USID 0x00u
#define PON_PID 0x08u
#define PON_SUBTYPE_OFF 0x05u
#define PON_SOFT_RB_SPARE_OFF 0x8fu
#define PON_BOOTLOADER_VAL 0x04u
#define PSCI_SYSTEM_RESET 0x84000009u

#define TRB_HWO 1u
#define TRB_LAST 2u
#define TRB_IOC (1u << 11)
#define TRB_SHORT (1u << 10)

struct trb { uint32_t low, high, size, control; };
// Separate cache lines prevent CPU cache maintenance from losing adjacent DMA writes.
struct dma_area {
    uint32_t events[EVENT_BYTES / 4];
    struct trb trbs[EP_COUNT][4];
    uint8_t setup[64];
    uint8_t control[512];
    uint8_t transmit[512];
    uint8_t receive[512];
} __attribute__((aligned(4096)));
static struct dma_area dma;
static uintptr_t registers;
static uint64_t dma_physical, frequency;
static bool active, configured, busy, direction_in, three_stage, status_started, initialization_attempted;
static bool qcom_usb0, qcom_usb_board_init;
static unsigned ep0_state, event_position, packet_size = 512, command_position;
static uint32_t transfer_length[EP_COUNT], resources[EP_COUNT];
static bool transfer_active[EP_COUNT];
static uint8_t ring[RING_BYTES];
static uint64_t ring_head, ring_tail, transfer_end, reset_deadline;
volatile uint32_t unixnu_dwc3_phase, unixnu_dwc3_error, unixnu_dwc3_last_event;
volatile uint32_t unixnu_dwc3_last_command, unixnu_dwc3_last_command_status;
volatile uint64_t unixnu_dwc3_transmitted, unixnu_dwc3_dropped;
volatile uint32_t unixnu_dwc3_qcom_session_before, unixnu_dwc3_qcom_session_after;
volatile uint32_t unixnu_dwc3_board_stage, unixnu_dwc3_board_error;
volatile uint32_t unixnu_dwc3_board_gcc, unixnu_dwc3_board_phy, unixnu_dwc3_board_smmu;
// Bring-up visibility without a console: the boot stage, painted as a coloured band on the
// board's own framebuffer, and the PMIC reset state.
volatile uint32_t unixnu_dwc3_stage, unixnu_qcom_reset_stage, unixnu_qcom_reset_error;
volatile uint32_t unixnu_qcom_reset_spare_before, unixnu_qcom_reset_spare_after;
static uint64_t fb_base, fb_mapped_address, fb_mapped_page;
static uint32_t fb_width, fb_height, fb_rowbytes;
static bool fb_mapped;

// Board reset and stage markers: the marker routine is called from reset_bootloader, which the
// host test compiles too, so both branches see the same prototypes.
int unixnu_qcom_mark_bootloader(void);
int unixnu_qcom_mark_bootloader_mapped(void);
static bool reset_initialize(void);
static void paint_stage(uint32_t stage);

#ifdef UNIXNU_DWC3_HOST_TEST
static uint64_t ticks(void);
static void cache_sync(void *, size_t);
static uint32_t read_reg(unsigned);
static void write_reg(unsigned, uint32_t);
static void reset_bootloader(void);
int unixnu_qcom_mark_bootloader(void) { return -1; }
int unixnu_qcom_mark_bootloader_mapped(void) { return -1; }
static bool reset_initialize(void) { return false; }
static void paint_stage(uint32_t stage) { (void)stage; }
#else
static uint64_t ticks(void) { uint64_t value; __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(value)); return value; }
static void cache_sync(void *address, size_t bytes)
{
    uint64_t ctr; __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    uintptr_t line = 4ul << ((ctr >> 16) & 15), end = (uintptr_t)address + bytes;
    for (uintptr_t p = (uintptr_t)address & ~(line - 1); p < end; p += line)
        __asm__ volatile("dc civac, %0" :: "r"(p) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");
}
static uint32_t read_reg(unsigned offset) { return *(volatile uint32_t *)(registers + offset); }
static void write_reg(unsigned offset, uint32_t value)
{
    *(volatile uint32_t *)(registers + offset) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
static void reset_bootloader(void)
{
    // PM660 PON_SOFT_RB_SPARE carries the bootloader reason: without the marker a PSCI
    // SYSTEM_RESET is a normal reboot and the firmware boots the system again. Mark first;
    // a board that cannot be marked still resets, just not into fastboot.
    unixnu_qcom_mark_bootloader();
    register uint64_t x0 __asm__("x0") = PSCI_SYSTEM_RESET;
    register uint64_t x1 __asm__("x1") = 0, x2 __asm__("x2") = 0, x3 __asm__("x3") = 0;
    __asm__ volatile("smc #0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3) : "memory");
}
#endif

#ifndef UNIXNU_DWC3_HOST_TEST
/* Board reset that returns to the bootloader. PSCI SYSTEM_RESET alone is a normal reboot: the
 * bootloader takes its reason from PM660 PON_SOFT_RB_SPARE (SPMI USID 0, PID 0x08, offset 0x8f),
 * and reason 0 boots the system again. Writing (spare & 1) | (mode-bootloader 2 << 1) = 0x04
 * first makes the next ABL stop in fastboot. This is the sequence package/BootReset.c verified on
 * this board under XNU 10063, including its guards: arbiter v5.*, PON owned by AP, subtype 0x04.
 * Enabling it is opt-in through /chosen/unixnu,qcom-reset, so a board whose firmware handles the
 * reset differently keeps the plain PSCI behaviour. */
static volatile uint32_t *spmi_obs, *spmi_rw;
static int spmi_channel = -1;
static uint32_t spmi_owner = 0xff, spmi_version;
static uint8_t spmi_subtype;
static bool spmi_ready, reset_initialized, reset_enabled;

static int spmi_read(uint8_t offset, uint8_t *value)
{
    if (!spmi_obs) return -1;
    spmi_obs[SPMI_REG_CONFIG / 4] = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    spmi_obs[SPMI_REG_CMD0 / 4] = ((uint32_t)SPMI_CMD_READ_LONG << 27) | ((uint32_t)offset << 4);
    __asm__ volatile("dsb sy" ::: "memory");
    uint32_t status = 0;
    for (unsigned tries = 0; tries < 200000; ++tries) {
        status = spmi_obs[SPMI_REG_STATUS / 4];
        if (status) break;
    }
    if (status != 1u) return -2;
    *value = (uint8_t)(spmi_obs[SPMI_REG_RDATA / 4] & 0xffu);
    return 0;
}

static int spmi_write(uint8_t offset, uint8_t value)
{
    if (!spmi_rw || spmi_version < 0x50000000u || spmi_version >= 0x70000000u ||
        spmi_owner != 0 || spmi_subtype != 0x04) return -1;
    spmi_rw[SPMI_REG_CONFIG / 4] = 0;
    spmi_rw[SPMI_REG_WDATA / 4] = (uint32_t)value;
    __asm__ volatile("dsb sy" ::: "memory");
    spmi_rw[SPMI_REG_CMD0 / 4] = ((uint32_t)SPMI_CMD_WRITE_LONG << 27) | ((uint32_t)offset << 4);
    __asm__ volatile("dsb sy" ::: "memory");
    uint32_t status = 0;
    for (unsigned tries = 0; tries < 200000; ++tries) {
        status = spmi_rw[SPMI_REG_STATUS / 4];
        if (status) break;
    }
    return status == 1u ? 0 : -2;
}

static bool reset_initialize(void)
{
    if (reset_initialized) return reset_enabled;
    reset_initialized = true;
    DTEntry chosen; const void *property; unsigned bytes;
    if (SecureDTLookupEntry(NULL, "/chosen", &chosen) != kSuccess ||
        SecureDTGetProperty(chosen, "unixnu,qcom-reset", &property, &bytes) != kSuccess ||
        bytes != sizeof(uint32_t)) return false;
    uint32_t enabled;
    memcpy(&enabled, property, sizeof(enabled));
    if (enabled != 1) return false;
    reset_enabled = true;
    unixnu_qcom_reset_stage = 1;
    volatile uint32_t *core = (volatile uint32_t *)ml_io_map(SPMI_CORE_PHYS, SPMI_WINDOW);
    volatile uint32_t *cnfg = (volatile uint32_t *)ml_io_map(SPMI_CNFG_PHYS, SPMI_WINDOW);
    if (!core || !cnfg) { unixnu_qcom_reset_error = 1; return false; }
    spmi_version = core[SPMI_ARB_VERSION / 4];
    uint32_t channels = core[SPMI_ARB_FEATURES / 4] & 0x7ffu;
    if (!channels || channels > 512) channels = 512;
    const uint32_t map = (spmi_version >= 0x50000000u) ? SPMI_APID_MAP_V5 : SPMI_APID_MAP_V3;
    for (uint32_t i = 0; i < channels; ++i) {
        uint32_t peripheral = core[(map + 4u * i) / 4u];
        if (((peripheral >> 16) & 0xfu) != PON_USID || ((peripheral >> 8) & 0xffu) != PON_PID) continue;
        uint32_t owner = cnfg[i] & 0x7u;
        if (spmi_channel < 0 || owner == 0) {
            spmi_channel = (int)i;
            spmi_owner = owner;
            if (owner == 0) break;
        }
    }
    if (spmi_channel < 0) { unixnu_qcom_reset_error = 2; return false; }
    const uint64_t observed = (uint64_t)SPMI_OBS_PHYS + (uint64_t)spmi_channel * 0x80ULL;
    vm_offset_t page = ml_io_map((vm_offset_t)(observed & ~0xfffULL), SPMI_WINDOW);
    if (page) spmi_obs = (volatile uint32_t *)(page + (observed & 0xfffULL));
    page = ml_io_map((vm_offset_t)((uint64_t)SPMI_CHNLS_PHYS + (uint64_t)spmi_channel * 0x10000ULL), SPMI_WINDOW);
    if (page) spmi_rw = (volatile uint32_t *)page;
    uint8_t subtype = 0xff;
    if (spmi_read(PON_SUBTYPE_OFF, &subtype) == 0) spmi_subtype = subtype;
    uint8_t spare = 0xff;
    (void)spmi_read(PON_SOFT_RB_SPARE_OFF, &spare);
    unixnu_qcom_reset_spare_before = spare;
    if (!spmi_obs || !spmi_rw) { unixnu_qcom_reset_error = 3; return false; }
    spmi_ready = true;
    return true;
}

/* The PMIC marker: returns 0 only when the bootloader reason reads back as written. */
int unixnu_qcom_mark_bootloader(void)
{
    if (!reset_initialize()) return -1;
    const int result = unixnu_qcom_mark_bootloader_mapped();
    if (result == 0) unixnu_qcom_reset_stage = 2;
    else if (!unixnu_qcom_reset_error) unixnu_qcom_reset_error = 4 + (uint32_t)-result;
    return result;
}

/* The same marker for panic context: it never maps, so a panic that happens before serial_init
 * mapped the PMIC leaves the reason alone and the reset is the plain PSCI one. */
int unixnu_qcom_mark_bootloader_mapped(void)
{
    if (!spmi_ready) return -1;
    uint8_t before = 0xff, after = 0xff;
    if (spmi_read(PON_SOFT_RB_SPARE_OFF, &before) != 0) return -1;
    const uint8_t wanted = (uint8_t)((before & 0x01u) | PON_BOOTLOADER_VAL);
    if (spmi_write(PON_SOFT_RB_SPARE_OFF, wanted) != 0) return -1;
    if (spmi_read(PON_SOFT_RB_SPARE_OFF, &after) != 0) return -1;
    unixnu_qcom_reset_spare_before = before;
    unixnu_qcom_reset_spare_after = after;
    return (after & 0xfeu) == PON_BOOTLOADER_VAL ? 0 : -1;
}

/* Mark and reset. Does not return once PSCI accepts the call. */
void unixnu_qcom_reset_to_bootloader(void)
{
    (void)unixnu_qcom_mark_bootloader();
    unixnu_qcom_reset_stage = 3;
    // dsb only: the component's counter audit requires every isb to be paired with a counter
    // read, and this barrier is about ordering the SPMI writes before the SMC, not timing.
    __asm__ volatile("dsb sy" ::: "memory");
    reset_bootloader();
    for (;;) __asm__ volatile("wfe");
}

/* Bring-up progress without a console: fill one horizontal band of the board's framebuffer.
 * Band N is the top (N+1)/16th, so how far the kernel got is visible as how much colour the
 * panel shows. Colours are distinct per stage; band 0 is never painted so the loader's own
 * diagnostics stay. The geometry comes from the ADT's display node, already validated by the
 * payload step; a missing or implausible framebuffer leaves the function inert. */
static void paint_stage(uint32_t stage)
{
    if (!fb_base || !fb_width || !fb_height || !fb_rowbytes || stage >= 12) return;
    // One 16x16 marker per stage along the bottom edge: which markers exist is how far the
    // kernel got. Only that strip is ever touched, so only the strip is mapped (the whole panel
    // would not fit the kernel's I/O submap).
    const uint32_t size = 16u, gap = 8u, margin = 16u, rows = size;
    const uint32_t top = fb_height > margin + rows ? fb_height - margin - rows : 0;
    const uint32_t left = margin + stage * (size + gap);
    if (left + size > fb_width) return;
    const uint64_t strip_first = (uint64_t)top * fb_rowbytes;
    const uint64_t strip_last = (uint64_t)(top + rows) * fb_rowbytes;
    const uint64_t page = strip_first & ~0xfffULL;
    if (!fb_mapped) {
        const vm_size_t bytes = (vm_size_t)(((strip_last - page) + 0xfffULL) & ~0xfffULL);
        vm_offset_t mapped = ml_io_map((vm_offset_t)(fb_base + page), bytes);
        if (!mapped) return;
        fb_mapped_address = (uint64_t)mapped;
        fb_mapped_page = page;
        fb_mapped = true;
    }
    const uint32_t colour = 0xff000000u | ((stage * 0x2b6d5u) & 0xffffffu);
    for (uint32_t row = top; row < top + rows; ++row)
        for (uint32_t col = left; col < left + size; ++col) {
            const uint64_t offset = (uint64_t)row * fb_rowbytes + (uint64_t)col * 4u;
            if (offset < fb_mapped_page) continue;
            *(volatile uint32_t *)(fb_mapped_address + (offset - fb_mapped_page)) = colour;
        }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void locate_framebuffer(const void *display, unsigned bytes)
{
    // /chosen/unixnu,display: base (u64), width, height, rowbytes (u32 each).
    if (!display || bytes != 20) return;
    uint64_t base;
    memcpy(&base, display, sizeof(base));
    const uint8_t *rest = (const uint8_t *)display + sizeof(base);
    uint32_t width, height, rowbytes;
    memcpy(&width, rest, 4); memcpy(&height, rest + 4, 4); memcpy(&rowbytes, rest + 8, 4);
    if (!base || !width || !height || rowbytes < width * 4u || height < 64) return;
    fb_base = base; fb_width = width; fb_height = height; fb_rowbytes = rowbytes;
    fb_mapped = false; fb_mapped_address = 0; fb_mapped_page = 0;
}
#endif

#ifndef UNIXNU_DWC3_HOST_TEST
/* Board preparation for a controller that firmware did not leave ready. The register sequence is
 * the one package/sirius_usb.c proved on the same SDM710: USB30 GDSC, its AXI/master/sleep/mock
 * branches, the QUSB2 v2 PHY, UTMI as the PIPE clock, and a direct route for SID 0x740 in the apps
 * SMMU. It is opt-in so the generic console still works on boards whose firmware did all of it. */
static uint32_t board_read(uintptr_t base, unsigned offset)
{
    return *(volatile uint32_t *)(base + offset);
}
static void board_write(uintptr_t base, unsigned offset, uint32_t value)
{
    *(volatile uint32_t *)(base + offset) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
static void board_delay_us(uint32_t microseconds)
{
    const uint64_t delta = frequency / 1000000u * (uint64_t)microseconds;
    const uint64_t deadline = ticks() + delta;
    while (ticks() < deadline) __asm__ volatile("yield" ::: "memory");
}
static bool board_branch_enable(uintptr_t gcc, unsigned offset)
{
    board_write(gcc, offset, board_read(gcc, offset) | 1u);
    const uint64_t deadline = ticks() + frequency / 10;
    while (board_read(gcc, offset) & (1u << 31))
        if (ticks() >= deadline) return false;
    return true;
}
static bool qcom_smmu_identity_route(uintptr_t smmu, uint64_t dma, uint64_t bytes)
{
    uint32_t scr0 = board_read(smmu, 0x000), idr0 = board_read(smmu, 0x020);
    uint32_t idr1 = board_read(smmu, 0x024), idr2 = board_read(smmu, 0x028);
    unsigned slots = idr0 & 0xffu, banks = idr1 & 0xffu;
    unsigned pages = 1u << (((idr1 >> 28) & 7u) + 1u);
    if (!(scr0 & 1u)) return true; // SMMU bypassed globally.
    if (!slots || slots > 128 || !banks || banks > 128 || pages > 128 ||
        (pages + banks) * 4096u > APPS_SMMU_BYTES) return false;

    unsigned matches = 0, free_slot = 0xffffu;
    uint32_t match_s2cr = 0;
    for (unsigned i = 0; i < slots; i++) {
        uint32_t smr = board_read(smmu, 0x800u + 4u * i);
        if (!(smr & (1u << 31))) {
            if (free_slot == 0xffffu) free_slot = i;
            continue;
        }
        uint32_t mask = (smr >> 16) & 0x7fffu;
        if (((smr ^ USB_STREAM_ID) & 0xffffu & ~mask) == 0 && mask == 0) {
            matches++;
            match_s2cr = board_read(smmu, 0xc00u + 4u * i);
        }
    }
    if (!matches) {
        if (free_slot == 0xffffu) return false;
        uint8_t bank_users[128] = {0};
        for (unsigned i = 0; i < slots; i++) {
            uint32_t smr = board_read(smmu, 0x800u + 4u * i);
            uint32_t s2cr = board_read(smmu, 0xc00u + 4u * i);
            if ((smr & (1u << 31)) && ((s2cr >> 16) & 3u) == 0 && (s2cr & 0xffu) < banks)
                bank_users[s2cr & 0xffu]++;
        }
        bool bound = false;
        for (unsigned attempt = 0; attempt < banks; attempt++) {
            unsigned bank = banks - 1u - attempt;
            if (bank_users[bank]) continue;
            uint32_t cbar = board_read(smmu, 0x1000u + 4u * bank);
            uint32_t cba2r = board_read(smmu, 0x1800u + 4u * bank);
            uint32_t sctlr = board_read(smmu, (pages + bank) * 4096u);
            if ((sctlr & 1u) || (cbar != 0 && cbar != 0x10000u)) continue;
            if (cbar != 0x10000u) board_write(smmu, 0x1000u + 4u * bank, 0x10000u);
            board_write(smmu, 0x1800u + 4u * bank, cba2r | 1u);
            board_write(smmu, 0xc00u + 4u * free_slot, bank);
            board_write(smmu, 0x800u + 4u * free_slot, 0x80000000u | USB_STREAM_ID);
            match_s2cr = board_read(smmu, 0xc00u + 4u * free_slot);
            if (board_read(smmu, 0x800u + 4u * free_slot) == (0x80000000u | USB_STREAM_ID) &&
                match_s2cr == bank) {
                matches = 1;
                bound = true;
            }
            break;
        }
        if (!bound) return false;
    }
    if (matches != 1) return false;
    const unsigned type = (match_s2cr >> 16) & 3u, bank = match_s2cr & 0xffu;
    if (type == 1) {
        /* The identity route still has to cover the console's DMA range. */
    } else if (type == 0 && bank < banks) {
        uint32_t cbar = board_read(smmu, 0x1000u + 4u * bank);
        uint32_t sctlr = board_read(smmu, (pages + bank) * 4096u);
        if (((cbar >> 16) & 3u) != 1u || (sctlr & 1u)) return false;
    } else return false;

    unsigned widths[3] = {0};
    const unsigned codes[3] = {idr2 & 0xf, (idr2 >> 4) & 0xf, (idr2 >> 8) & 0xf};
    for (unsigned i = 0; i < 3; i++) {
        static const unsigned bits[] = {32, 36, 40, 42, 44, 48};
        if (codes[i] < sizeof(bits) / sizeof(bits[0])) widths[i] = bits[codes[i]];
        if (!widths[i] || dma >= (1ULL << widths[i]) || bytes > (1ULL << widths[i]) - dma) return false;
    }
    return true;
}
static bool qcom_board_prepare(void)
{
    uintptr_t gcc = ml_io_map(GCC_PHYS, GCC_BYTES);
    unixnu_dwc3_board_stage = 1;
    if (!gcc) { unixnu_dwc3_board_error = 1; return false; }
    uint32_t gdscr = board_read(gcc, GCC_USB30_PRIM_GDSCR);
    if (!(gdscr & (1u << 31))) {
        board_write(gcc, GCC_USB30_PRIM_GDSCR, gdscr & ~1u);
        const uint64_t deadline = ticks() + frequency / 10;
        while (!(board_read(gcc, GCC_USB30_PRIM_GDSCR) & (1u << 31)))
            if (ticks() >= deadline) { unixnu_dwc3_board_error = 2; return false; }
    }
    unixnu_dwc3_board_stage = 2;
    static const unsigned branches[] = {GCC_CFG_NOC_USB3_PRIM, GCC_AGGRE_USB3_PRIM,
                                        GCC_USB30_PRIM_MASTER, GCC_USB30_PRIM_SLEEP,
                                        GCC_USB30_PRIM_MOCK_UTMI};
    for (unsigned i = 0; i < sizeof(branches) / sizeof(branches[0]); i++)
        if (!board_branch_enable(gcc, branches[i])) {
            unixnu_dwc3_board_error = 3 + i;
            return false;
        }
    unixnu_dwc3_board_gcc = board_read(gcc, GCC_USB30_PRIM_GDSCR);
    unixnu_dwc3_board_stage = 3;

    /* The QUSB2 PHY's AHB and CXO reference branches must be running before any PHY read. */
    if (!board_branch_enable(gcc, GCC_USB_PHY_CFG_AHB2PHY) ||
        !board_branch_enable(gcc, GCC_USB3_PRIM_CLKREF)) {
        unixnu_dwc3_board_error = 8;
        return false;
    }
    board_write(gcc, GCC_QUSB2PHY_PRIM_BCR, 1u);
    board_delay_us(100);
    board_write(gcc, GCC_QUSB2PHY_PRIM_BCR, 0u);
    board_delay_us(100);

    uintptr_t phy = ml_io_map(QUSB2_PHY_PHYS, QUSB2_PHY_BYTES);
    unixnu_dwc3_board_stage = 4;
    if (!phy) { unixnu_dwc3_board_error = 9; return false; }
    static const struct { uint32_t offset, value; } init[] = {
        {0x04, 0x03}, {0x18c, 0x7c}, {0x2c, 0x80}, {0x184, 0x0a}, {0xb4, 0x19},
        {0x194, 0x40}, {0x198, 0x20}, {0x214, 0x21}, {0x220, 0x00}, {0x224, 0x58},
        {0x240, 0x30}, {0x244, 0x29}, {0x248, 0xca}, {0x24c, 0x04}, {0x250, 0x03},
        {0x23c, 0x00},
    };
    board_write(phy, QUSB2_PHY_POWERDOWN, board_read(phy, QUSB2_PHY_POWERDOWN) | 0x23u);
    board_write(phy, QUSB2_PHY_PLL_CORE_INPUT_OVERRIDE,
                board_read(phy, QUSB2_PHY_PLL_CORE_INPUT_OVERRIDE) | 0x70u);
    for (unsigned i = 0; i < sizeof(init) / sizeof(init[0]); i++) board_write(phy, init[i].offset, init[i].value);
    board_write(phy, QUSB2_PHY_POWERDOWN, board_read(phy, QUSB2_PHY_POWERDOWN) & ~1u);
    board_delay_us(200);
    board_write(phy, QUSB2_PHY_PLL_CORE_INPUT_OVERRIDE,
                board_read(phy, QUSB2_PHY_PLL_CORE_INPUT_OVERRIDE) & ~0x60u);
    board_delay_us(200);
    const uint64_t lock_deadline = ticks() + frequency / 10;
    bool locked = false;
    while (ticks() < lock_deadline) {
        if ((board_read(phy, 0x1a4) & QUSB2_PHY_CORE_READY) ||
            (board_read(phy, 0x1ac) & QUSB2_PHY_PLL_LOCKED) ||
            (board_read(phy, QUSB2_PHY_PLL_STATUS) & QUSB2_PHY_PLL_LOCKED)) {
            locked = true;
            break;
        }
        board_delay_us(1000);
    }
    unixnu_dwc3_board_phy = board_read(phy, QUSB2_PHY_PLL_STATUS);
    if (!locked) { unixnu_dwc3_board_error = 10; return false; }
    unixnu_dwc3_board_stage = 5;

    uintptr_t smmu = ml_io_map(APPS_SMMU_PHYS, APPS_SMMU_BYTES);
    if (!smmu) { unixnu_dwc3_board_error = 11; return false; }
    if (!qcom_smmu_identity_route(smmu, dma_physical, sizeof(dma))) {
        unixnu_dwc3_board_error = 12;
        unixnu_dwc3_board_smmu = board_read(smmu, 0x020);
        return false;
    }
    unixnu_dwc3_board_smmu = board_read(smmu, 0x020);
    unixnu_dwc3_board_stage = 6;
    return true;
}
#endif
static void qcom_select_utmi_pipe_clock(void)
{
    uint32_t cfg = read_reg(QSCRATCH_GENERAL_CFG);
    write_reg(QSCRATCH_GENERAL_CFG, cfg | QSCRATCH_PIPE_UTMI_CLK_DIS);
    write_reg(QSCRATCH_GENERAL_CFG, (cfg | QSCRATCH_PIPE_UTMI_CLK_DIS |
                                    QSCRATCH_PIPE_UTMI_CLK_SEL | QSCRATCH_PIPE3_PHYSTATUS_SW));
    write_reg(QSCRATCH_GENERAL_CFG, (cfg | QSCRATCH_PIPE_UTMI_CLK_SEL | QSCRATCH_PIPE3_PHYSTATUS_SW) &
                                    ~QSCRATCH_PIPE_UTMI_CLK_DIS);
}

static uint64_t physical(void *p) { return dma_physical + (uintptr_t)p - (uintptr_t)&dma; }
static bool command(unsigned ep, unsigned cmd, uint32_t p0, uint32_t p1, uint32_t p2)
{
    if (ep >= EP_COUNT) return false;
    write_reg(DEP(ep), p2); write_reg(DEP(ep) + 4, p1); write_reg(DEP(ep) + 8, p0);
    unixnu_dwc3_last_command = (ep << 16) | cmd;
    write_reg(DEP(ep) + 12, cmd | (1u << 10));
    uint64_t deadline = ticks() + frequency / 100; // 10 ms, no scheduler dependency
    uint32_t result;
    do {
        result = read_reg(DEP(ep) + 12);
        if (!(result & (1u << 10))) {
            unixnu_dwc3_last_command_status = result;
            if (result & 0xf000) { unixnu_dwc3_error = 2; return false; }
            if ((cmd & 15) == 6) resources[ep] = (result >> 16) & 127;
            return true;
        }
    } while (ticks() < deadline);
    unixnu_dwc3_error = 1;
    return false;
}
static void end_transfer(unsigned ep)
{
    if (transfer_active[ep]) (void)command(ep, 8 | (1u << 11) | (resources[ep] << 16), 0, 0, 0);
    transfer_active[ep] = false; resources[ep] = 0;
}
static bool endpoint(unsigned ep, unsigned type, unsigned packet, bool modify)
{
    uint32_t p0 = (type << 1) | (packet << 3) | ((ep & 1) ? ((ep >> 1) << 17) : 0);
    if (modify) p0 |= 2u << 30;
    uint32_t p1 = (ep << 25) | (1u << 8) | ((type == 0) ? (1u << 10) : (1u << 9));
    if (type == 3) p1 |= 8u << 16;
    if (!command(ep, 1, p0, p1, 0)) return false;
    write_reg(DALEPENA, read_reg(DALEPENA) | (1u << ep));
    return true;
}
static bool transfer(unsigned ep, void *buffer, unsigned bytes, unsigned type)
{
    if (ep >= EP_COUNT || bytes > 512 || transfer_active[ep]) return false;
    struct trb *trb = &dma.trbs[ep][0];
    uint64_t pa = physical(buffer), tp = physical(trb);
    trb->low = (uint32_t)pa; trb->high = (uint32_t)(pa >> 32); trb->size = bytes;
    trb->control = (type << 4) | TRB_HWO | TRB_LAST | TRB_IOC | TRB_SHORT;
    cache_sync(buffer, bytes); cache_sync(trb, 64);
    if (!command(ep, 6, (uint32_t)(tp >> 32), (uint32_t)tp, 0)) return false;
    transfer_length[ep] = bytes; transfer_active[ep] = true;
    return true;
}
static void arm_setup(void)
{
    end_transfer(0); end_transfer(1);
    ep0_state = 0; status_started = false;
    (void)transfer(0, dma.setup, 8, 2);
}
static void status_transfer(unsigned ep)
{
    if (status_started || ep0_state == 0) return;
    status_started = true; ep0_state = 2;
    (void)transfer(ep, dma.control, 0, three_stage ? 4 : 3);
}
static void stall_setup(void)
{
    (void)command(0, 4, 0, 0, 0);
    arm_setup();
}

static const uint8_t device_descriptor[] = {18,1,0,2,2,0,0,64,0x25,5,0xa7,0xa4,0,2,1,2,3,1};
static uint8_t configuration_descriptor[] = {
    9,2,67,0,2,1,0,0xc0,50, 9,4,0,0,1,2,2,0,0,
    5,0x24,0,0x10,1, 5,0x24,1,0,1, 4,0x24,2,2, 5,0x24,6,0,1,
    7,5,0x82,3,16,0,9, 9,4,1,0,2,0xa,0,0,0,
    7,5,0x81,2,0,2,0, 7,5,1,2,0,2,0
};
static const uint8_t qualifier[] = {10,6,0,2,2,0,0,64,1,0};
static const uint8_t languages[] = {4,3,9,4};
static const char *const strings[] = {"", "UniXNU", "zeta XNU log", "0"};
static uint8_t line_coding[7] = {0,0xc2,1,0,0,0,8}; // 115200, 8N1; USB transport is independent

static void control_in(const void *bytes, unsigned length, unsigned requested)
{
    if (length > requested) length = requested;
    if (length > sizeof(dma.control)) { stall_setup(); return; }
    memcpy(dma.control, bytes, length);
    ep0_state = 1;
    (void)transfer(1, dma.control, length, 5);
}
static void handle_setup(void)
{
    cache_sync(dma.setup, sizeof(dma.setup));
    const uint8_t *s = dma.setup;
    unsigned type = s[0], request = s[1], value = s[2] | (s[3] << 8);
    unsigned index = s[4] | (s[5] << 8), length = s[6] | (s[7] << 8);
    direction_in = (type & 128) != 0; three_stage = length != 0;
    status_started = false; ep0_state = length ? 1 : 2;
    if (type == 0x80 && request == 6) {
        unsigned descriptor = value >> 8, number = value & 255;
        if (descriptor == 1 && number == 0) control_in(device_descriptor, sizeof(device_descriptor), length);
        else if (descriptor == 2 && number == 0) control_in(configuration_descriptor, sizeof(configuration_descriptor), length);
        else if (descriptor == 6 && number == 0) control_in(qualifier, sizeof(qualifier), length);
        else if (descriptor == 3 && number == 0) control_in(languages, sizeof(languages), length);
        else if (descriptor == 3 && number < sizeof(strings)/sizeof(strings[0])) {
            uint8_t text[128]; unsigned count = 0;
            while (strings[number][count] && count < 63) { text[2 + 2*count] = (uint8_t)strings[number][count]; text[3 + 2*count] = 0; count++; }
            text[0] = (uint8_t)(2 + count*2); text[1] = 3; control_in(text, text[0], length);
        } else stall_setup();
    } else if (type == 0 && request == 5 && !length && value < 128) {
        write_reg(DCFG, (read_reg(DCFG) & ~(127u << 3)) | (value << 3));
    } else if (type == 0 && request == 9 && !length && value <= 1) {
        configured = false; end_transfer(2); end_transfer(3); end_transfer(5);
        write_reg(DALEPENA, 3);
        if (value == 1) {
            configured = endpoint(2, 2, packet_size, false) && endpoint(3, 2, packet_size, false) && endpoint(5, 3, 16, false);
            if (configured) { unixnu_dwc3_phase = 6; (void)transfer(2, dma.receive, 512, 1); }
            else stall_setup();
        }
    } else if (type == 0x80 && request == 8 && length == 1) {
        uint8_t config = configured ? 1 : 0; control_in(&config, 1, length);
    } else if ((type == 0x80 || type == 0x81 || type == 0x82) && request == 0 && length == 2) {
        uint8_t result[2] = {type == 0x80 ? 1 : 0, 0}; control_in(result, 2, length);
    } else if (type == 2 && request == 1 && value == 0 && !length && (index & 15) <= 2) {
        unsigned ep = ((index & 15) << 1) | ((index >> 7) & 1);
        (void)command(ep, 5, 0, 0, 0);
    } else if (type == 0x81 && request == 10 && length == 1 && index < 2) {
        uint8_t alt = 0; control_in(&alt, 1, length);
    } else if (type == 1 && request == 11 && !length && !value && index < 2) {
        // Only alternate setting zero exists.
    } else if (type == 0xa1 && request == 0x21 && index == 0 && length == 7) {
        control_in(line_coding, 7, length);
    } else if (type == 0x21 && request == 0x20 && index == 0 && length == 7) {
        (void)transfer(0, dma.control, 64, 5);
    } else if (type == 0x21 && (request == 0x22 || request == 0x23) && index == 0 && !length) {
        // Control lines/break do not change the byte transport.
    } else stall_setup();
}

static void append(uint8_t byte)
{
    if (ring_head - ring_tail == RING_BYTES) { ring_tail++; unixnu_dwc3_dropped++; }
    ring[ring_head++ % RING_BYTES] = byte;
}
static void send_pending(void)
{
    if (!configured || transfer_active[3] || ring_head == ring_tail) return;
    unsigned bytes = (unsigned)(ring_head - ring_tail);
    if (bytes > sizeof(dma.transmit)) bytes = sizeof(dma.transmit);
    for (unsigned i = 0; i < bytes; i++) dma.transmit[i] = ring[(ring_tail + i) % RING_BYTES];
    if (transfer(3, dma.transmit, bytes, 1)) transfer_end = ring_tail + bytes;
}
static void receive_command(unsigned length)
{
    static const uint8_t command_bytes[] = "\0ZETA:REBOOT-BOOTLOADER\n";
    cache_sync(dma.receive, sizeof(dma.receive));
    for (unsigned i = 0; i < length; i++) {
        uint8_t c = dma.receive[i];
        if (c == command_bytes[command_position]) {
            if (++command_position == sizeof(command_bytes) - 1) {
                static const char reply[] = "XNUUSB: returning to bootloader\n";
                command_position = 0;
                for (unsigned j = 0; j < sizeof(reply) - 1; j++) append((uint8_t)reply[j]);
                reset_deadline = ticks() + frequency / 5;
            }
        } else command_position = c == 0 ? 1 : 0;
    }
}
static void endpoint_event(uint32_t event)
{
    unsigned ep = (event >> 1) & 31, kind = (event >> 6) & 15, status = (event >> 12) & 15;
    if (ep >= EP_COUNT) return;
    if (kind == 3 && ep <= 1 && (status & 3) == 2) { status_transfer(ep); return; }
    if (kind != 1 && kind != 2) return;
    if (!transfer_active[ep]) return;
    cache_sync(&dma.trbs[ep][0], 64);
    if (dma.trbs[ep][0].control & TRB_HWO) return;
    unsigned remaining = dma.trbs[ep][0].size & 0xffffff;
    if (remaining > transfer_length[ep]) { unixnu_dwc3_error = 3; return; }
    unsigned completed = transfer_length[ep] - remaining;
    transfer_active[ep] = false; resources[ep] = 0;
    if (ep <= 1) {
        if (ep0_state == 0) handle_setup();
        else if (ep0_state == 2) arm_setup();
        else {
            if (!direction_in && completed == 7) { cache_sync(dma.control, sizeof(dma.control)); memcpy(line_coding, dma.control, 7); }
            // Status is armed only upon the controller's XferNotReady status event.
            ep0_state = 2;
        }
    } else if (ep == 3) {
        uint64_t completed_end = transfer_end - remaining;
        unixnu_dwc3_transmitted += completed;
        if (completed_end > ring_tail) ring_tail = completed_end;
        unixnu_dwc3_phase = 7;
    } else if (ep == 2) {
        receive_command(completed);
        if (configured) (void)transfer(2, dma.receive, 512, 1);
    }
}

void unixnu_dwc3_poll(void)
{
    if (!active || __atomic_exchange_n(&busy, true, __ATOMIC_ACQUIRE)) return;
    unsigned bytes = read_reg(GEVNTCOUNT) & 0xfffc;
    if (bytes > EVENT_BYTES) { unixnu_dwc3_error = 5; bytes = EVENT_BYTES; }
    if (bytes > 256) bytes = 256; // Bound work in a timer interrupt.
    cache_sync(dma.events, EVENT_BYTES);
    for (unsigned i = 0; i < bytes; i += 4) {
        uint32_t event = dma.events[event_position / 4]; event_position = (event_position + 4) % EVENT_BYTES;
        unixnu_dwc3_last_event = event;
        if (!(event & 1)) endpoint_event(event);
        else if (((event >> 1) & 127) == 0) {
            unsigned kind = (event >> 8) & 15;
            if (kind == 0 || kind == 1) {
                configured = false; command_position = 0;
                for (unsigned ep = 0; ep < EP_COUNT; ep++) end_transfer(ep);
                write_reg(DCFG, read_reg(DCFG) & ~(127u << 3));
                if (kind == 1) arm_setup();
            } else if (kind == 2) {
                unixnu_dwc3_phase = 5;
                packet_size = ((read_reg(DSTS) & 7) == 0) ? 512 : 64;
                configuration_descriptor[57] = (uint8_t)packet_size; configuration_descriptor[58] = (uint8_t)(packet_size >> 8);
                configuration_descriptor[64] = (uint8_t)packet_size; configuration_descriptor[65] = (uint8_t)(packet_size >> 8);
                (void)endpoint(0, 0, 64, true); (void)endpoint(1, 0, 64, true);
            }
        }
        write_reg(GEVNTCOUNT, 4);
    }
    send_pending();
    if (reset_deadline && (ring_tail == ring_head || ticks() >= reset_deadline)) reset_bootloader();
    __atomic_store_n(&busy, false, __ATOMIC_RELEASE);
}
unsigned int unixnu_dwc3_ready(void) { unixnu_dwc3_poll(); return 1; }
void unixnu_dwc3_put(uint8_t byte) { append(byte); unixnu_dwc3_poll(); }
unsigned int unixnu_dwc3_receive_ready(void) { unixnu_dwc3_poll(); return 0; }
uint8_t unixnu_dwc3_receive(void) { return 0; }

// Diagnostic: /chosen/unixnu,dwc3-probe-reset-ms (u32, milliseconds) makes this function wait and
// then reset the board through PSCI SYSTEM_RESET, so a boot with no console still reports how far
// the kernel got: the wait is measured from the first PE_init_kprintf that calls serial_init, which
// is after the platform expert, VM and IOKit initialization. No default: without the property this
// is inert.
#ifndef UNIXNU_DWC3_HOST_TEST
static void unixnu_probe_reset(uint32_t milliseconds)
{
    uint64_t frequency;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    uint64_t deadline = 0;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(deadline));
    deadline += frequency / 1000 * (uint64_t)milliseconds;
    for (;;) {
        uint64_t now;
        __asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
        if (now >= deadline) break;
    }
    // The bootloader needs its PMIC reason set first; otherwise this is a normal reboot.
    unixnu_qcom_mark_bootloader();
    register uint64_t function __asm__("x0") = PSCI_SYSTEM_RESET;
    __asm__ volatile("smc #0" :: "r"(function) : "memory", "x1", "x2", "x3");
    for (;;) { __asm__ volatile("wfi"); }
}
#endif

bool unixnu_dwc3_init(void)
{
#ifndef UNIXNU_DWC3_HOST_TEST
    {
        // Before every other check: report through the reset, not through USB.
        DTEntry probe_chosen; const void *probe_value; unsigned probe_bytes;
        if (SecureDTLookupEntry(NULL, "/chosen", &probe_chosen) == kSuccess &&
            SecureDTGetProperty(probe_chosen, "unixnu,dwc3-probe-reset-ms", &probe_value, &probe_bytes) == kSuccess &&
            probe_bytes == sizeof(uint32_t)) {
            uint32_t milliseconds; memcpy(&milliseconds, probe_value, sizeof(milliseconds));
            if (milliseconds) unixnu_probe_reset(milliseconds);
        }
    }
#endif
    // serial_init is called again by PE_init_kprintf. Preserve live transfers,
    // buffered output and an enumeration in progress instead of resetting USB.
    if (initialization_attempted) return active;
#ifndef UNIXNU_DWC3_HOST_TEST
    DTEntry chosen; const void *property; unsigned bytes;
    if (SecureDTLookupEntry(NULL, "/chosen", &chosen) != kSuccess) return false;
    // Report this call on the panel before anything that can fail: serial_init is reached right
    // after the MMU is on and arm_vm_init returns, so a band here means the console path ran.
    if (SecureDTGetProperty(chosen, "unixnu,display", &property, &bytes) == kSuccess) locate_framebuffer(property, bytes);
    unixnu_dwc3_stage = 1;
    paint_stage(unixnu_dwc3_stage);
    // Map the PMIC now, while a panic cannot be in progress: the panic screen resets through it,
    // and ml_io_map is not a call to make with interrupts masked in panic context.
    (void)reset_initialize();
    if (SecureDTGetProperty(chosen, "unixnu,dwc3-console", &property, &bytes) != kSuccess || bytes != 16) return false;
    uint64_t window[2]; memcpy(window, property, sizeof(window));
    if (!window[0] || window[0] % 4096 || window[1] < 0xd000 || window[1] > 0x100000) return false;
    if (SecureDTGetProperty(chosen, "unixnu,qcom-usb0", &property, &bytes) == kSuccess) {
        uint32_t enabled;
        if (bytes != sizeof(enabled)) return false;
        memcpy(&enabled, property, sizeof(enabled));
        if (enabled != 1 || window[0] != 0x0a600000 || window[1] < 0xf8814) return false;
        qcom_usb0 = true;
    }
    if (SecureDTGetProperty(chosen, "unixnu,qcom-usb-board-init", &property, &bytes) == kSuccess) {
        uint32_t enabled;
        if (bytes != sizeof(enabled)) return false;
        memcpy(&enabled, property, sizeof(enabled));
        if (enabled != 1 || !qcom_usb0) return false;
        qcom_usb_board_init = true;
    }
    initialization_attempted = true;
    registers = ml_io_map((vm_offset_t)window[0], (vm_size_t)window[1]);
    if (!registers) return false;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    dma_physical = ml_static_vtop((vm_offset_t)&dma);
    unixnu_dwc3_stage = 2;
    paint_stage(unixnu_dwc3_stage);
    if (qcom_usb_board_init && !qcom_board_prepare()) return false;
    if (qcom_usb_board_init) { unixnu_dwc3_stage = 3; paint_stage(unixnu_dwc3_stage); }
#endif
    initialization_attempted = true;
    if (!frequency || !dma_physical) return false;
    unixnu_dwc3_phase = 1;
    uint32_t id = read_reg(GSNPSID) >> 16;
    if (id != 0x5533 && id != 0x3331) return false;
    write_reg(DCTL, (read_reg(DCTL) & ~(1u << 31)) | (1u << 30));
    uint64_t deadline = ticks() + frequency / 10;
    while (read_reg(DCTL) & (1u << 30)) if (ticks() >= deadline) { unixnu_dwc3_error = 6; return false; }
    unixnu_dwc3_phase = 2;
    write_reg(GCTL, (read_reg(GCTL) & ~((3u << 12) | (3u << 4) | (1u << 1) | (1u << 3))) |
              (2u << 12) | (qcom_usb0 ? 1u : 0)); // lahaina DT: disable-clk-gating
    // Keep firmware's UTMI width and PHY tuning; clear USB2 PHY suspend/sleep gating.
    write_reg(0xc200, read_reg(0xc200) & ~((1u << 6) | (1u << 8)));
    if (qcom_usb_board_init) qcom_select_utmi_pipe_clock();
    if (qcom_usb0) {
        // QSCRATCH HS_PHY_CTRL, as in dwc3-qcom's VBUS override.
        // USB is attached to a host; select and assert UTMI session validity.
        unixnu_dwc3_qcom_session_before = read_reg(QSCRATCH_HS_PHY_CTRL);
        write_reg(QSCRATCH_HS_PHY_CTRL, unixnu_dwc3_qcom_session_before |
                  QSCRATCH_UTMI_OTG_VBUS_VALID | QSCRATCH_SW_SESSVLD_SEL);
        unixnu_dwc3_qcom_session_after = read_reg(QSCRATCH_HS_PHY_CTRL);
    }
    memset(&dma, 0, sizeof(dma)); cache_sync(&dma, sizeof(dma));
    write_reg(GEVNTADRLO, (uint32_t)dma_physical); write_reg(GEVNTADRHI, (uint32_t)(dma_physical >> 32));
    write_reg(GEVNTSIZ, EVENT_BYTES | (1u << 31)); write_reg(GEVNTCOUNT, read_reg(GEVNTCOUNT) & 0xfffc);
    write_reg(DCFG, read_reg(DCFG) & ~((127u << 3) | 7u)); // High speed, address zero
    write_reg(DEVTEN, 7); write_reg(DALEPENA, 0);
    unixnu_dwc3_stage = 4;
    paint_stage(unixnu_dwc3_stage);
    if (!command(0, 9, 0, 0, 0)) return false;
    for (unsigned ep = 0; ep < EP_COUNT; ep++) if (!command(ep, 2, 1, 0, 0)) return false;
    if (!endpoint(0, 0, 64, false) || !endpoint(1, 0, 64, false)) return false;
    unixnu_dwc3_phase = 3;
    arm_setup();
    if (!transfer_active[0]) return false;
    active = true;
    unixnu_dwc3_stage = 5;
    paint_stage(unixnu_dwc3_stage);
    write_reg(DCTL, read_reg(DCTL) | (1u << 31));
    unixnu_dwc3_phase = 4;
    deadline = ticks() + frequency * 8;
    while (!configured && ticks() < deadline) unixnu_dwc3_poll();
    if (configured) {
        unixnu_dwc3_stage = 6;
        paint_stage(unixnu_dwc3_stage);
        static const char banner[] = "XNUUSB: native DWC3 console ready\n";
        for (unsigned i = 0; i < sizeof(banner) - 1; i++) append((uint8_t)banner[i]);
        uint64_t flush = ticks() + frequency / 2;
        while (ring_head != ring_tail && ticks() < flush) unixnu_dwc3_poll();
    }
    // Register the byte sink even if Linux enumerates later. Early boot text
    // stays in the ring and polling continues from output and timer callbacks.
    return active;
}
