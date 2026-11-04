#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
# libfirehose_kernel.a for the kernel link (LD_KERNEL_ARCHIVES), arm64 like the kernel.
# The fakeroot and the XNU working tree belong to the build that asks for it: the xnu-10063
# baseline keeps the defaults, build-xnu-12377.sh passes its versioned ones (UNIXNU_FAKEROOT,
# UNIXNU_XNU_WORK) and the deployment target of its OS profile (UNIXNU_FIREHOSE_IOS_MIN).
FAKEROOT=${UNIXNU_FAKEROOT:-$ROOT/.cache/fakeroot}
XNU_WORK=${UNIXNU_XNU_WORK:-$ROOT/.cache/xnu-work}
IOS_MIN=${UNIXNU_FIREHOSE_IOS_MIN:-17.0}
if [ ! -d .cache/libdispatch-work/.git ]; then
  git clone --shared refer/libdispatch .cache/libdispatch-work
fi
UNIXNU_XNU_WORK="$XNU_WORK" python3 - <<'PY'
import os
from pathlib import Path
p = Path('.cache/libdispatch-work/xcodeconfig/libfirehose_kernel.xcconfig')
s = p.read_text()
s = s.replace('$(SDKROOT)/System/Library/Frameworks/Kernel.framework',
              '$(FAKEROOT_DIR)/System/Library/Frameworks/Kernel.framework')
s = s.replace('$(SDKROOT)/usr/local/include', '$(FAKEROOT_DIR)/usr/local/include')
lines = s.splitlines()
for index, line in enumerate(lines):
    if line.startswith('HEADER_SEARCH_PATHS =') and '$(SDKROOT)/usr/include' not in line:
        lines[index] += (' $(SDKROOT)/usr/include $(FAKEROOT_DIR)/usr/local/include'
                         ' ' + os.path.abspath(os.environ['UNIXNU_XNU_WORK']) + '/EXTERNAL_HEADERS')
s = '\n'.join(lines) + '\n'
if p.read_text() != s:
    p.write_text(s)
PY
if [ "$(uname -s)" = Linux ]; then
  # No Xcode: build the target as its xcconfig describes it. libfirehose_kernel is one source file and one
  # private header (installed through unifdef with COPY_HEADERS_UNIFDEF_FLAGS); symbols stay external, as
  # Xcode's GCC_SYMBOLS_PRIVATE_EXTERN=NO leaves them.
  WORK=.cache/libdispatch-work
  mkdir -p build/firehose/obj "$FAKEROOT/usr/local/lib/kernel" "$FAKEROOT/usr/local/include/kernel/os"
  xcrun --sdk iphoneos clang -target "arm64-apple-ios$IOS_MIN" -std=gnu11 -Os -g \
    -mkernel -nostdinc -Wno-packed \
    -DKERNEL=1 -DDISPATCH_USE_DTRACE=0 -DOS_ATOMIC_CONFIG_MEMORY_ORDER_DEPENDENCY=1 \
    -DOS_ATOMIC_CONFIG_STARVATION_FREE_ONLY=0 \
    -I"$WORK" -I"$FAKEROOT/System/Library/Frameworks/Kernel.framework/PrivateHeaders" \
    -I"$FAKEROOT/System/Library/Frameworks/Kernel.framework/Headers" \
    -I"$FAKEROOT/usr/local/include/os" -I"$FAKEROOT/usr/local/include/firehose" \
    -I"$FAKEROOT/usr/include" -I"$FAKEROOT/usr/local/include" -I"$XNU_WORK/EXTERNAL_HEADERS" \
    -c "$WORK/src/firehose/firehose_buffer.c" -o build/firehose/obj/firehose_buffer.o
  rm -f "$FAKEROOT/usr/local/lib/kernel/libfirehose_kernel.a"
  xcrun libtool -static -o "$FAKEROOT/usr/local/lib/kernel/libfirehose_kernel.a" build/firehose/obj/firehose_buffer.o
  unifdef -DKERNEL=1 -DOS_FIREHOSE_SPI=1 -DOS_VOUCHER_ACTIVITY_SPI_TYPES=1 -UOS_VOUCHER_ACTIVITY_SPI \
    -o "$FAKEROOT/usr/local/include/kernel/os/firehose_buffer_private.h" \
    "$WORK/os/firehose_buffer_private.h" || [ $? -eq 1 ]
else
  xcodebuild install -project .cache/libdispatch-work/libdispatch.xcodeproj \
    -target libfirehose_kernel -configuration Release -sdk iphoneos -jobs "${JOBS:-4}" \
    ARCHS=arm64 VALID_ARCHS=arm64 ONLY_ACTIVE_ARCH=NO IPHONEOS_DEPLOYMENT_TARGET="$IOS_MIN" \
    CODE_SIGNING_ALLOWED=NO GCC_TREAT_WARNINGS_AS_ERRORS=NO \
    OBJROOT="$ROOT/build/firehose/obj" SYMROOT="$ROOT/build/firehose/sym" \
    DSTROOT="$FAKEROOT" FAKEROOT_DIR="$FAKEROOT"
fi
if [ -f "$FAKEROOT/usr/local/lib/kernel/liblibfirehose_kernel.a" ]; then
  cp "$FAKEROOT/usr/local/lib/kernel/liblibfirehose_kernel.a" \
    "$FAKEROOT/usr/local/lib/kernel/libfirehose_kernel.a"
fi
xcrun lipo -info "$FAKEROOT/usr/local/lib/kernel/libfirehose_kernel.a"
