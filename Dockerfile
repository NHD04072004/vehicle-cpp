# TARGETARCH: BuildKit gán amd64|arm64 theo máy (không default — tránh build nhầm arch).
# Ghi đè: --build-arg TARGETARCH=arm64 --build-arg BASE_IMAGE=nhd04072004/ds_app:8.0-arm64
ARG TARGETARCH
ARG BASE_IMAGE=nhd04072004/ds_app:8.0-${TARGETARCH}


## Build stage
FROM ${BASE_IMAGE} AS build

RUN apt-get update && apt-get install -y --no-install-recommends \
      libjsoncpp-dev \
      libcurl4-openssl-dev \
      libjpeg-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY CMakeLists.txt vehicle.sh main.cpp ./
COPY src/ ./src/
COPY resources/ds/nvdsinfer_custom_impl_Yolo/ ./resources/ds/nvdsinfer_custom_impl_Yolo/
COPY resources/ds/nvdsinfer_custom_impl_Yolo_pose/ ./resources/ds/nvdsinfer_custom_impl_Yolo_pose/
COPY resources/ds/nvdspreprocess_custom_warp_perspective/ ./resources/ds/nvdspreprocess_custom_warp_perspective/
RUN bash vehicle.sh build -DBUILD_TESTS=OFF

## Production stage
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends \
      libmosquitto1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=build /src/build/vehicle /app/build/vehicle
COPY --from=build /src/build/libs/libnvdsinfer_custom_impl_Yolo.so /app/build/libs/
COPY --from=build /src/build/libs/libnvdsinfer_custom_impl_Yolo_pose.so /app/build/libs/
COPY --from=build /src/build/libs/libnvdspreprocess_custom_warp_perspective.so /app/build/libs/
COPY resources/config/ /app/resources/config/
COPY resources/ds/ /app/resources/ds/
COPY vehicle.sh /app/vehicle.sh
RUN chmod +x /app/vehicle.sh

ENV VEHICLE_ROOT=/app \
    TZ=Asia/Ho_Chi_Minh \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,video \
    USE_NEW_NVSTREAMMUX=yes

ENTRYPOINT ["/app/vehicle.sh"]
