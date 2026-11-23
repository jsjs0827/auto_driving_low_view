# Release variants may omit linker PDBs. Copy a real source SDK PDB when present.
if(EXISTS "${SOURCE}" AND IS_DIRECTORY "${DESTINATION}")
  file(COPY "${SOURCE}" DESTINATION "${DESTINATION}")
endif()
