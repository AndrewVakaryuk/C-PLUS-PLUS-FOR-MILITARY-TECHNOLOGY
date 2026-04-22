FROM ubuntu:noble

ARG DEBIAN_FRONTEND="noninteractive"

# Packages
COPY apt-packages.in /tmp/apt.in
RUN apt update && \
  grep -vE '^\s*#' /tmp/apt.in | xargs apt install --no-install-recommends -y && \
  rm -rf /var/lib/apt/lists/* && apt-get clean -y && rm /tmp/apt.in && \
  sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
  locale-gen

# CPP config files
COPY .clang-format /.clang-format
COPY .clangd /.clangd
COPY .cmake-format.json /.cmake-format.json

ENV TZ=Etc/UTC
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en

RUN echo "ALL ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG CONTAINER_USER=developer
ARG WORKSPACE_FOLDER=/workspace

RUN userdel -r ubuntu 2>/dev/null || true && \
    groupdel ubuntu 2>/dev/null || true && \
    useradd -s /bin/bash -m -u 1000 -U ${CONTAINER_USER} && \
  echo "\nexport PROMPT_COMMAND=\"history -a; history -n\n\"" >> /home/${CONTAINER_USER}/.bashrc

USER ${CONTAINER_USER}
