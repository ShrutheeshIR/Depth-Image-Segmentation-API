static PyObject* pybv_bv_segmentdepthimage(PyObject* , PyObject* args, PyObject* kw)
{
    using namespace bv;

    {
    PyObject* pyobj_depth_image = NULL;
    Mat depth_image;
    PyObject* pyobj_camera_matrix = NULL;
    Mat camera_matrix;
    PyObject* pyobj_imedge = NULL;
    Mat imedge;
    int retval;

    const char* keywords[] = { "depth_image", "camera_matrix", "imedge", NULL };
    if( PyArg_ParseTupleAndKeywords(args, kw, "OO|O:segmentdepthimage", (char**)keywords, &pyobj_depth_image, &pyobj_camera_matrix, &pyobj_imedge) &&
        pybv_to(pyobj_depth_image, depth_image, ArgInfo("depth_image", 0)) &&
        pybv_to(pyobj_camera_matrix, camera_matrix, ArgInfo("camera_matrix", 0)) &&
        pybv_to(pyobj_imedge, imedge, ArgInfo("imedge", 1)) )
    {
        ERRWRAP2(retval = bv::segmentdepthimage(depth_image, camera_matrix, imedge));
        return Py_BuildValue("(NN)", pybv_from(retval), pybv_from(imedge));
    }
    }
    PyErr_Clear();

    {
    PyObject* pyobj_depth_image = NULL;
    Mat depth_image;
    PyObject* pyobj_camera_matrix = NULL;
    Mat camera_matrix;
    PyObject* pyobj_imedge = NULL;
    UMat imedge;
    int retval;

    const char* keywords[] = { "depth_image", "camera_matrix", "imedge", NULL };
    if( PyArg_ParseTupleAndKeywords(args, kw, "OO|O:segmentdepthimage", (char**)keywords, &pyobj_depth_image, &pyobj_camera_matrix, &pyobj_imedge) &&
        pybv_to(pyobj_depth_image, depth_image, ArgInfo("depth_image", 0)) &&
        pybv_to(pyobj_camera_matrix, camera_matrix, ArgInfo("camera_matrix", 0)) &&
        pybv_to(pyobj_imedge, imedge, ArgInfo("imedge", 1)) )
    {
        ERRWRAP2(retval = bv::segmentdepthimage(depth_image, camera_matrix, imedge));
        return Py_BuildValue("(NN)", pybv_from(retval), pybv_from(imedge));
    }
    }

    return NULL;
}

