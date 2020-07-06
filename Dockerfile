FROM archlinux
LABEL maintainer="Wil Thomason,wbthomason at cs dot cornell dot edu"
RUN useradd -m build && usermod -L build
RUN pacman -Syu --needed --noconfirm go git sudo base-devel
# NOTE: This is insecure if you have any reason to care about security for this container
RUN echo "build ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/build
USER build
WORKDIR /home/build
RUN mkdir .gnupg && \
    touch .gnupg/gpg.conf && \
    echo "keyserver-options auto-key-retrieve" > .gnupg/gpg.conf
RUN git clone https://aur.archlinux.org/yay.git yay-build
WORKDIR yay-build
RUN makepkg --noconfirm --syncdeps --rmdeps --install --clean
RUN yay -S --needed --noconfirm meson luajit clang spdlog cxxopts boost fmt ompl bullet nlopt \
      urdfdom urdfdom-headers tinyxml2 eigen nlohmann-json
USER root
WORKDIR /
RUN userdel -r build
RUN git clone https://github.com/wbthomason-robotics/planet 
WORKDIR planet
RUN git submodule update --init --recursive
RUN ./build.sh release
