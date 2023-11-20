FROM debian:bookworm

ADD image/setup.sh /image/
RUN /image/setup.sh

ADD image/build.sh /image/
ADD image/drake-src.tar /image/drake/
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/bazel \
    /image/build.sh
