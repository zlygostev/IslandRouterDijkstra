#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <vector>
#include <functional>
#include <ostream>

namespace visualizer {

/// Pixel values
enum ImagePixelValues
{
    IPV_PATH = 0,               ///< Results are red in the BMP
    IPV_WATER = 1,              ///< Results are the water color in the BMP
 	IPV_QUICK_PATH = 2,         ///< Results are green in the BMP
	IPV_NORMAL_PATH = 3,        ///< Results are green in the BMP
	IPV_SLOW_PATH = 4,          ///< Results are yellow in the BMP
	IPV_TOO_SLOW_PATH = 5,      ///< Results are red in the BMP
	IPV_ELEVATION_BEGIN = 6,    ///< 6-255

};


/** A method to write BMP file contents to a specified ostream.
 *
 * @param out The ostream to use for output. Could be directed into anything
 * @param elevationData Pointer to grid of elevation values. There must be width * height such
 *        elevation points.
 * @param width The width of the image
 * @param height The height of the image
 * @param pixelFilter A passed function or lambda that can change the pixel colormap index at passed
 *        x (from the left), and y (from the top) position of the elevationData. See enum
 *        ImagePixelValues above for interesting values to return.
 */
void writeBMP(
    std::ostream& out,
    const uint8_t* elevationData,
    size_t width,
    size_t height,
    std::function<uint8_t(size_t, size_t, uint8_t)> pixelFilter);

} // namespace visualizer

#endif // __VISUALIZER_H__
