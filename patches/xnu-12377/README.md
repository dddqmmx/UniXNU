# xnu-12377 patch series (iOS 26)

The series that brings the public `xnu-12377.121.6` tree (Darwin 25.5.0, the source release
closest to iOS 26.5.2 build 23F84) to the DEVELOPMENT BCM2837 ARM64 configuration, applied in
file-name order by `scripts/build-xnu-12377.sh`. Its build is the iOS 26 kernel of a
non-PAC board and is separate from `patches/xnu`, which serves the xnu-10063 (iPadOS 17) build.

Provenance: taken from `none24smu/UniXNU` branch `fix/cache-batch-kernel` (commit
`35c384e84266343991fd14706e8c651f7d2e09ed`), the series UniXNU-zeta shipped on the Surface Duo 2
(SM8350, also without pointer authentication). All 44 patches were verified to apply cleanly and
in order to a pristine `xnu-12377.121.6` checkout (`ac9718fb1af618d5ce8678d0dc6e8a58f252216f`).

Board-specific patches in this series, which a new port should keep but not rely on:

| Patch | Note |
| --- | --- |
| `native-ufs-owner-root.patch`, `native-storage-personalities.patch` | UFS storage; an eMMC board uses its own driver |
| `zzzz-native-gpu-addon-mount.patch`, `zzzz-native-adreno-sandbox-grant.patch` | Adreno GPU addon of the zeta port |
| `native-dwc3-console.patch` | polling USB console; a board with its own transport keeps it off |

The two patches a non-PAC board needs for an iOS 26 userland are
`native-shared-region-small-pages.patch` (4 KiB slide pages) and
`native-shared-region-plain-auth.patch` (store the slid plain pointer, which is what a lowered
arm64 userland expects).
