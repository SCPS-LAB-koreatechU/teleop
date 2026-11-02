# Check the mesh directories exist before install() trips over them.
#
# Three of the four are symlinks into content that is fetched rather than
# committed -- the assets submodule and the vendored DexHand descriptions -- so
# a fresh clone has dangling links. `install(DIRECTORY ...)` then fails with
#
#   ament_cmake_symlink_install_directory() can't find '.../meshes/'
#
# which names the path and nothing else, and says nothing about submodules. This
# turns each case into the command that fixes it.

function(require_mesh_directory name remedy)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${name}/")
    message(FATAL_ERROR
      "\n"
      "${name}/ is missing, or is a symlink whose target is not there.\n"
      "\n"
      "  ${remedy}\n"
      "\n"
      "Run that from the repository root, then build again.")
  endif()
endfunction()

function(require_mesh_directories)
  require_mesh_directory(meshes
    "git submodule update --init source/berkeley_humanoid_lite_assets")
  require_mesh_directory(meshes_dexhand "ros2_ws/fetch_vendor.sh")
  require_mesh_directory(meshes_v1 "ros2_ws/fetch_vendor.sh")
  require_mesh_directory(meshes_dexhand_left
    "python3 ros2_ws/src/berkeley_humanoid_lite_description/mirror_meshes.py")
endfunction()
