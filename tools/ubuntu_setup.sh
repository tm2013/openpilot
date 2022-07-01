#!/bin/bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$(cd $DIR/../ && pwd)"

# NOTE: this is used in a docker build, so do not run any scripts here.

# Install packages present in all supported versions of Ubuntu
function install_ubuntu_common_requirements() {
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    autoconf \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    make \
    cppcheck \
    libtool \
    gcc-arm-none-eabi \
    bzip2 \
    liblzma-dev \
    libarchive-dev \
    libbz2-dev \
    capnproto \
    libcapnp-dev \
    curl \
    libcurl4-openssl-dev \
    git \
    git-lfs \
    ffmpeg \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libavfilter-dev \
    libeigen3-dev \
    libffi-dev \
    libglew-dev \
    libgles2-mesa-dev \
    libglfw3-dev \
    libglib2.0-0 \
    libomp-dev \
    libopencv-dev \
    libpng16-16 \
    libssl-dev \
    libsqlite3-dev \
    libusb-1.0-0-dev \
    libzmq3-dev \
    libsystemd-dev \
    locales \
    opencl-headers \
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    clinfo \
    qml-module-qtquick2 \
    qtmultimedia5-dev \
    qtlocation5-dev \
    qtpositioning5-dev \
    qttools5-dev-tools \
    libqt5sql5-sqlite \
    libqt5svg5-dev \
    libqt5x11extras5-dev \
    libreadline-dev \
    libdw1 \
    valgrind
}

# Install Ubuntu 22.04 LTS packages
function install_ubuntu_latest_requirements() {
  install_ubuntu_common_requirements

  sudo apt-get install -y --no-install-recommends \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    python3-dev
}

# Install Ubuntu 20.04 packages
function install_ubuntu_lts_requirements() {
  install_ubuntu_common_requirements

  sudo apt-get install -y --no-install-recommends \
    libavresample-dev \
    qt5-default \
    python-dev
}


# Install packages present in all supported versions of Fedora
function install_fedora_common_requirements() {
  sudo dnf check-update
  sudo dnf install -y --setopt=install_weak_deps=False \
    autoconf \
    make automake gcc gcc-c++ kernel-devel glibc patch \
    ca-certificates \
    clang \
    cmake \
    make \
    cppcheck \
    libtool \
    arm-none-eabi-gcc-cs-c++ \
    bzip2 \
    lzma-sdk457-devel \
    libarchive-devel \
    bzip2-devel \
    capnproto \
    capnproto-devel \
    curl \
    libcurl-devel \
    git \
    git-lfs \
    ffmpeg \
    ffmpeg-devel \
    eigen3-devel \
    libffi-devel \
    glew-devel \
    mesa-libGL-devel \
    glfw-devel \
    glib2-devel \
    libomp-devel \
    opencv-devel \
    libpng \
    openssl-devel \
    sqlite-devel \
    libusb1-devel \
    zeromq-devel \
    systemd-devel \
    opencl-headers \
    ocl-icd \
    ocl-icd-devel \
    clinfo \
    qt5-qtquickcontrols2 \
    qt5-qtmultimedia-devel \
    qt5-qtlocation-devel \
    qt5-qtbase \
    qt5-qtsvg-devel \
    qt5-qtx11extras-devel \
    readline-devel \
    elfutils-libs \
    valgrind \
    ffmpeg-libs \
    qt5-qtbase-devel \
    qtchooser \
    python3-devel
}

# Install Fedora 36 packages
function install_fedora_36_requirements() {
  install_fedora_common_requirements
}


# Detect OS using /etc/os-release file
if [ -f "/etc/os-release" ]; then
  source /etc/os-release
  case "$ID $VERSION_ID" in
    "ubuntu 22.04")
      install_ubuntu_latest_requirements
      ;;
    "ubuntu 20.04")
      install_ubuntu_lts_requirements
      ;;
    "fedora 36")
      echo "$ID $VERSION_ID is unsupported and OP build fails. This setup script is written for Ubuntu 20.04."
      read -p "Would you like to attempt installation anyway? " -n 1 -r
      echo ""
      if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
      fi
      install_fedora_36_requirements
      ;;
    *)
      echo "$ID $VERSION_ID is unsupported. This setup script is written for Ubuntu 20.04."
      read -p "Would you like to attempt installation anyway? " -n 1 -r
      echo ""
      if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
      fi
      install_ubuntu_lts_requirements
      ;;
  esac
else
  echo "No /etc/os-release in the system"
  exit 1
fi


# install python dependencies
$ROOT/update_requirements.sh

source ~/.bashrc
if [ -z "$OPENPILOT_ENV" ]; then
  printf "\nsource %s/tools/openpilot_env.sh" "$ROOT" >> ~/.bashrc
  source ~/.bashrc
  echo "added openpilot_env to bashrc"
fi

echo
echo "----   OPENPILOT SETUP DONE   ----"
echo "Open a new shell or configure your active shell env by running:"
echo "source ~/.bashrc"
