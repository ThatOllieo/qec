# Installing libgpiod v2 on Raspberry Pi OS

Raspberry Pi OS (Bullseye and Bookworm) ships with libgpiod 1.6.x. The v2 API is not yet in the package repos, so it must be built from source.

## Prerequisites

```bash
sudo apt-get update
sudo apt-get install -y git build-essential autoconf automake libtool pkg-config autoconf-archive
```

## Build and Install

```bash
git clone https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
cd libgpiod
git checkout v2.1.3   # or run: git tag -l 'v2*' to see available versions

./autogen.sh --enable-tools --prefix=/usr
make -j$(nproc)
sudo make install
sudo ldconfig
```

## Fix pkg-config Path

The Pi's multiarch pkg-config path (`/usr/lib/arm-linux-gnueabihf/pkgconfig/`) takes precedence over `/usr/lib/pkgconfig/` where the new `.pc` file was installed. Overwrite the old one:

```bash
sudo cp /usr/lib/pkgconfig/libgpiod.pc /usr/lib/arm-linux-gnueabihf/pkgconfig/libgpiod.pc
```

## Verify

```bash
pkg-config --modversion libgpiod   # should print 2.x.x
gpiodetect --version               # should print 2.x.x
```

The installed shared library will be `libgpiod.so.3.x.x` — the `.so.3` soname is correct for v2; this is not the same as v1's `.so.2`.
