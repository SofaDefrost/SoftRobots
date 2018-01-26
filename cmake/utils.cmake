
# Add automatically Metis and PythonPlugin option
# And disable SOFA_USE_MASK
if(NOT PLUGIN_SOFAPYTHON)
    set(PLUGIN_SOFAPYTHON ON CACHE BOOL "Build the SofaPython Plugin." FORCE)
endif(NOT PLUGIN_SOFAPYTHON)

if(NOT SOFA_BUILD_METIS)
    set(SOFA_BUILD_METIS ON CACHE BOOL "Build the metis library distributed in the extlibs/ directory.  It is used only
                                        by the SparseLDLSolver component.  Note that this library is usually available
                                        on Linux distributions." FORCE)
endif(NOT SOFA_BUILD_METIS)

if(SOFA_USE_MASK)
    set(SOFA_USE_MASK OFF CACHE BOOL "Use mask optimization" FORCE)
endif(SOFA_USE_MASK)
