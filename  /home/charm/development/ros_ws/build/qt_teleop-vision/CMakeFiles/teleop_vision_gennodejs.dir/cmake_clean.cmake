file(REMOVE_RECURSE
  "labelling_node_automoc.cpp"
  "reporter_utils_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/teleop_vision_gennodejs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
