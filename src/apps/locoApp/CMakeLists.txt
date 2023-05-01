cmake_minimum_required(VERSION 3.11)

project(locoApp)

file(GLOB CRL_SOURCES #
        "*.h" #
        "*.cpp" #
        )

list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "crl::loco" #
)

list(
        APPEND
        CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC "crl::loco" #
)

create_crl_app(
        ${PROJECT_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)
