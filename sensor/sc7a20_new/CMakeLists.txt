cmake_minimum_required(VERSION 3.16)
project(sc7a20_new C)

option(CONFIG_USE_SYNC_API "Build sync API" ON)
option(CONFIG_USE_ASYNC_API "Build async API" ON)

add_library(sc7a20_new STATIC
    src/sc7a20_core.c
)

if(CONFIG_USE_SYNC_API)
    target_sources(sc7a20_new PRIVATE src/sc7a20_sync.c)
endif()

if(CONFIG_USE_ASYNC_API)
    target_sources(sc7a20_new PRIVATE src/sc7a20_async.c)
endif()

target_include_directories(sc7a20_new PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

add_executable(sc7a20_new_unit
    adapters/mock_adapter.c
    test/unit_test.c
)
target_link_libraries(sc7a20_new_unit PRIVATE sc7a20_new)

add_executable(sc7a20_new_integration
    adapters/mock_adapter.c
    test/integration_test.c
)
target_link_libraries(sc7a20_new_integration PRIVATE sc7a20_new)
