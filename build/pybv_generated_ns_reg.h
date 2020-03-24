static PyMethodDef methods_bv[] = {
    {"segmentdepthimage", (PyCFunction)pybv_bv_segmentdepthimage, METH_VARARGS | METH_KEYWORDS, "segmentdepthimage(depth_image, camera_matrix[, imedge]) -> retval, imedge\n."},
    {NULL, NULL}
};

static ConstDef consts_bv[] = {
    {NULL, 0}
};

static void init_submodules(PyObject * root) 
{
  init_submodule(root, MODULESTR"", methods_bv, consts_bv);
};
