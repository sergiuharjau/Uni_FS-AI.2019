#include <boost/python.hpp>
using namespace boost::python;

#include "threading.cpp"

BOOST_PYTHON_MODULE( expose_threading )
{
	class_<ZedCam, boost::noncopyable >( "ZedCam", init<>() )
		.def( init<>() )
		.def( "init", &ZedCam::init )
		.def( "capture", &ZedCam::capture)
		.def( "latestImage", &ZedCam::latestImage)
		.def( "latestDepth", &ZedCam::latestDepth)
		.def( "exit", &ZedCam::exit);
}
