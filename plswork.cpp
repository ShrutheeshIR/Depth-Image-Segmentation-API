#include <pyboostcvconverter/pyboostcvconverter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
namespace pbcvt {
using namespace std;

using namespace boost::python;
cv::Mat dot(PyObject *image) {

    cv::Mat matImage, greyMat;
    matImage = pbcvt::fromNDArrayToMat(image);
    cv::cvtColor(matImage, greyMat, CV_BGR2GRAY);
    return greyMat;
}
cv::Mat dot2(cv::Mat leftMat, cv::Mat rightMat) {
    auto c1 = leftMat.cols, r2 = rightMat.rows;
    if (c1 != r2) {
        PyErr_SetString(PyExc_TypeError,
                        "Incompatible sizes for matrix multiplication.");
        throw_error_already_set();
    }
    cv::Mat result = leftMat * rightMat;

    return result;
}


#if (PY_VERSION_HEX >= 0x03000000)
    static void *init_ar() {
#else
    static void init_ar(){
#endif
    Py_Initialize();

    import_array();
    return NUMPY_IMPORT_ARRAY_RETVAL;
}

BOOST_PYTHON_MODULE (pbcvt) {
    //using namespace XM;
    init_ar();

    //initialize converters
    to_python_converter<cv::Mat,
            pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();

    //expose module-level functions
    def("dot", dot);
    def("dot2", dot2);

    }

}