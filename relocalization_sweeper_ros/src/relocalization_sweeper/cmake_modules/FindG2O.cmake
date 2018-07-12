INCLUDE_DIRECTORIES("/usr/local/include")
SET(G2O_INCLUDE_DIRS "/usr/local/include")

LINK_DIRECTORIES("/usr/local/lib")
SET(G2O_LIB_DIR "/usr/local/lib")

SET(G2O_LIBS 
    libg2o_types_data.so;
    libg2o_types_slam2d.so;
    libg2o_types_icp.so;
    libg2o_solver_csparse.so;
    libg2o_simulator.so;
    libg2o_solver_eigen.so;
    libg2o_parser.so;
    libg2o_incremental.so;
    libg2o_core.so;
    libg2o_cli.so;
    libg2o_types_sim3.so;
    libg2o_solver_pcg.so;
    libg2o_solver_cholmod.so;
    libg2o_types_sba.so;
    libg2o_interface.so;
    libg2o_solver_slam2d_linear.so;
    libg2o_stuff.so;
    libg2o_types_slam3d.so;
    libg2o_solver_dense.so;
    libg2o_solver_structure_only.so;
    libg2o_types_slam2d_addons.so;
    libg2o_csparse_extension.so;
    libg2o_interactive.so;
    libg2o_types_slam3d_addons.so;
    libg2o_opengl_helper.so;
    libg2o_ext_freeglut_minimal.so;
    libg2o_types_sclam2d.so
  )

SET(G2O_FOUND YES)
SET(G2O_FOUND "YES")
