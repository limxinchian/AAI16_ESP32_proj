# Runtime and generator are unmodified nanopb 0.4.9.2; see vendor/nanopb/UPSTREAM.json.
# Bindings are checked in so firmware builds need no protoc or host ML packages.
idf_component_register(
    SRCS "aai16_protocol.c" "generated/aai16.pb.c"
         "vendor/nanopb/pb_common.c" "vendor/nanopb/pb_encode.c" "vendor/nanopb/pb_decode.c"
    INCLUDE_DIRS "include" "generated" "vendor/nanopb")
target_compile_definitions(${COMPONENT_LIB} PUBLIC PB_VALIDATE_UTF8=1 PB_MESSAGE_NESTING_MAX=8)
