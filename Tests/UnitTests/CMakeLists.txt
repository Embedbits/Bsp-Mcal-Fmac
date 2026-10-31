#======================== Unit tests configuration ============================#
# Unit tests of Fmac module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(FMAC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${FMAC_DIR}/.." ABSOLUTE)

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
                    ${MCAL_DIR}/Gpdma/Gpdma_Port.h
    INCLUDE_DIRS    ${FMAC_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpdma
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
