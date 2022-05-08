#include <cfloat>
#include <tgmath.h>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mr_geometry/world_scoped_maps.h>


using namespace moro;

WorldScopedMaps::WorldScopedMaps () :
    width_pixel_ ( -1 ),
    height_pixel_ ( -1 ),
    min_x_ ( 0 ), max_x_ ( 0 ), min_y_ ( 0 ), max_y_ ( 0 ) {

}

bool WorldScopedMaps::initialized() {
    return ( ( width_pixel_ != -1 ) && ( height_pixel_ != -1 ) );
}

void WorldScopedMaps::init () {
    /**
     * @ToDo Wanderer
     * you have to fill some local variables which you can use to create the transformation matrix
     * use max_x_, min_x_, max_y_, min_y_, width_pixel_, height_pixel_, rotation_ for the computation
     * This function transforms from the metric space of the sensor coordinate frame to the image coordinate frame
     * Checkout the slides about geometry
     **/
#if GEOMETRY_EXERCISE >= 10
#else
    dx_ = max_x_ - min_x_;  // visual width
    dy_ = max_y_ - min_y_;    // visual height
    sx_ = width_pixel_ / dx_;  // scaling x
    sy_ = height_pixel_ / dy_;  // scaling y
    ox_ = width_pixel_ / 2;  // offset image space x
    oy_ = height_pixel_ / 2;  // offset image space y
    mx_ = min_x_ + dx_ / 2.;  // visual image space x
    my_ = min_y_ + dy_ / 2.;  // visual image space y
    cv::Matx<double, 3, 3 > Tw ( 1, 0, -mx_, 0, 1, -my_, 0, 0, 1 ); // translation visual space
    cv::Matx<double, 3, 3 > Sc ( sx_, 0, 0, 0, sy_, 0, 0, 0, 1 ); // scaling
    cv::Matx<double, 3, 3 > Sp ( -1, 0, 0, 0, 1, 0, 0, 0, 1 ); // mirroring
    cv::Matx<double, 3, 3 > R ( cos(rotation_), -sin(rotation_), 0, sin(rotation_), cos(rotation_), 0, 0, 0, 1 );  // rotation
    cv::Matx<double, 3, 3 > Tm ( 1, 0, ox_, 0, 1, oy_, 0, 0, 1 ); // translation image space
    Mw2m_ = Tm * R * Sp * Sc * Tw;
    //Mw2m_ = cv::Matx<double, 3, 3 > ( 15, 3, 350, 2, 5, 400, 0, 0, 1 ); ///  @ToDo remove this line dummy matrix
#endif

    Mm2w_ = Mw2m_.inv();
}
void WorldScopedMaps::init ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double rotation ) {
    width_pixel_ = width_pixel,   height_pixel_ = height_pixel;
    min_y_ = std::min ( min_y, max_y );
    max_y_ = std::max ( min_y, max_y );
    min_x_ = std::min ( min_x, max_x );
    max_x_ = std::max ( min_x, max_x );
    rotation_ = rotation;

    init();
}

const cv::Matx33d  &WorldScopedMaps::Mw2m () const {
    return Mw2m_;
}
const cv::Matx33d  &WorldScopedMaps::Mm2w () const {
    return Mm2w_;
}
Point2D WorldScopedMaps::w2m ( const Point2D &src ) const {
    return Mw2m_ * src;
}
Point2D &WorldScopedMaps::w2m ( const Point2D &src, Point2D &des ) const {
    des = Mw2m_ * src;
    return des;
}
Point2D WorldScopedMaps::m2w ( const Point2D &src ) const {
    return Mm2w_ * src;
}
Point2D &WorldScopedMaps::m2w ( const Point2D &src, Point2D &des ) const {
    des = Mm2w_ * src;
    return des;
}

void WorldScopedMaps::line ( cv::Mat &view,  const Point2D &p0, const Point2D &p1, const cv::Scalar &color, int thickness, int lineType ) const {
    cv::line ( view, w2m ( p0 ).cv(), w2m ( p1 ).cv(), color, thickness, lineType );
}

void WorldScopedMaps::circle ( cv::Mat &view, const Point2D &p, int radius, const cv::Scalar &color, int thickness, int lineType ) const {
    cv::circle ( view, w2m ( p ).cv(), radius, color, thickness, lineType );
}

double WorldScopedMaps::max_x () const {
    return max_x_;
}
double WorldScopedMaps::min_x () const  {
    return min_x_;
}
double WorldScopedMaps::scale_x () const  {
    return sx_;
}
double WorldScopedMaps::max_y () const  {
    return max_y_;
}
double WorldScopedMaps::min_y () const  {
    return min_y_;
}
double WorldScopedMaps::scale_y () const  {
    return sy_;
}
int WorldScopedMaps::width () const {
    return width_pixel_;
}
int WorldScopedMaps::height () const {
    return height_pixel_;
}
