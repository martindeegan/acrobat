function(symlink_link_configs)
  # Set locations of lint config files
  get_filename_component(PARENT_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)
  set(CLANG_FORMAT_CONFIG_ORIGINAL ${PARENT_DIR}/.clang-format)
  set(CLANG_TIDY_CONFIG_ORIGINAL ${PARENT_DIR}/.clang-tidy)
  set(CMAKE_FORMAT_CONFIG_ORIGINAL ${PARENT_DIR}/.cmake-format.yaml)

  # Set locations of symlinked files
  set(LINT_CONFIG_SHARE_PATH
      ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config
      CACHE PATH "")
  set(CLANG_FORMAT_CONFIG_SYMLINK
      ${LINT_CONFIG_SHARE_PATH}/.clang-format
      CACHE FILEPATH "")
  set(CLANG_TIDY_CONFIG_SYMLINK
      ${LINT_CONFIG_SHARE_PATH}/.clang-tidy
      CACHE FILEPATH "")
  set(CMAKE_FORMAT_CONFIG_SYMLINK
      ${LINT_CONFIG_SHARE_PATH}/.cmake-format.yaml
      CACHE FILEPATH "")

  # Symlink our lint config files
  file(MAKE_DIRECTORY ${LINT_CONFIG_SHARE_PATH})
  file(CREATE_LINK ${CLANG_FORMAT_CONFIG_ORIGINAL} ${CLANG_FORMAT_CONFIG_SYMLINK} SYMBOLIC)
  file(CREATE_LINK ${CLANG_TIDY_CONFIG_ORIGINAL} ${CLANG_TIDY_CONFIG_SYMLINK} SYMBOLIC)
  file(CREATE_LINK ${CMAKE_FORMAT_CONFIG_ORIGINAL} ${CMAKE_FORMAT_CONFIG_SYMLINK} SYMBOLIC)

  # Return the filepaths to the symlinks
  set(CLANG_FORMAT_CONFIG
      ${CLANG_FORMAT_CONFIG_SYMLINK}
      PARENT_SCOPE)
  set(CLANG_TIDY_CONFIG
      ${CLANG_TIDY_CONFIG_SYMLINK}
      PARENT_SCOPE)
  set(CMAKE_FORMAT_CONFIG
      ${CMAKE_FORMAT_CONFIG_SYMLINK}
      PARENT_SCOPE)
endfunction()