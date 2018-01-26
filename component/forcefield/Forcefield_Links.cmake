
#target_link_libraries(${PROJECT_NAME} SofaRigid SofaMiscMapping)

# SofaRigid
find_package(SofaCommon REQUIRED)
# SofaMiscMapping
find_package(SofaMisc REQUIRED)
target_link_libraries(${PROJECT_NAME} SofaRigid SofaMiscMapping)


