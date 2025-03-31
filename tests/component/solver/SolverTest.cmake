

list(APPEND SOURCE_FILES

    component/solver/QPInverseProblemImplTest.cpp
    #component/solver/QPInverseProblemSolverTest.cpp

    )

if(PLUGIN_SOFAPYTHON3)
  list(APPEND SOURCE_FILES
      component/solver/QPInverseProblemSolverWithContactTest.cpp
  )
endif()
