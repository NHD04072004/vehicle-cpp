ARG BASE_IMAGE=nhd04072004/ds_app:8.0-amd64

## Build stage
FROM ${BASE_IMAGE} AS build

WORKDIR /src
COPY CMakeLists.txt vehicle.sh main.cpp ./
COPY src/ ./src/
COPY resources/ds/nvdsinfer_custom_impl_Yolo/ ./resources/ds/nvdsinfer_custom_impl_Yolo/
COPY resources/ds/nvdsinfer_custom_impl_Yolo_pose/ ./resources/ds/nvdsinfer_custom_impl_Yolo_pose/
COPY resources/ds/nvdspreprocess_custom_warp_perspective/ ./resources/ds/nvdspreprocess_custom_warp_perspective/
RUN bash vehicle.sh build -DBUILD_TESTS=OFF

## Production stage
FROM ${BASE_IMAGE}

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
