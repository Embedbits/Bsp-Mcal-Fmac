#======================== Unit tests configuration ============================#
# Unit tests of Fmac module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(FMAC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${FMAC_DIR}/.." ABSOLUTE)

# The DMA streams do not exist on STM32H7R / STM32H7S (no FMAC there either) - an empty Dma_Port.h is mocked there
if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")
    set(UT_DMA_MOCK_HEADER ${MCAL_DIR}/Dma/Tests/UnitTests/Stub/Dma_Port.h)
else()
    set(UT_DMA_MOCK_HEADER ${MCAL_DIR}/Dma/Dma_Port.h)
endif()

# The FMAC hardware reactions (reset, completion of a load function, the data written to FMAC_WDATA)
# are not emulated by plain RegMem: the tests activate the HW model UtFmac_HwModel (see the test source).
# Rcc, Nvic and Gpdma modules are mocked.
UnitTesting_Add_Test(
    NAME            Fmac
    TEST_SOURCE     Test_Fmac.c
    UUT_SOURCES     ${FMAC_DIR}/Fmac.c
                    ${FMAC_DIR}/Fmac_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${UT_DMA_MOCK_HEADER}
    INCLUDE_DIRS    ${FMAC_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Dma
                    ${MCAL_DIR}/Gpdma                       # DMA types of STM32H7R / STM32H7S
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
