//boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//standard
#include <iostream>

//VisionWorkbench
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/InterestPoint/InterestData.h>

using namespace vw;
using namespace vw::ip;

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

template <class ImageT, class ValueT>
void draw_line( ImageViewBase<ImageT>& image,
                ValueT const& value,
                Vector2i const& start,
                Vector2i const& end ) {

  BBox2i bound = bounding_box(image.impl());
  if ( !bound.contains( start ) ||
       !bound.contains( end ) )
    return;
  Vector2i delta = end - start;
  for ( float r=0; r<1.0; r+=1/norm_2(delta) ) {
    int i = (int)(0.5 + start.x() + r*float(delta.x()) );
    int j = (int)(0.5 + start.y() + r*float(delta.y()) );
    image.impl()(i,j) = value;
  }
}

int main( int argc, char *argv[] ){

  std::vector<std::string> input_file_names;
  std::string output_prefix;
  float scalar;

  po::options_description general_options("Options");
  general_options.add_options()
    ("output-prefix,o",po::value<std::string>(&output_prefix)->default_value("color_ip_"), "Output prefix")
    ("reduce,r",po::value<float>(&scalar)->default_value(2), "Scalar to reduce image by")
    ("help,h","Brings up this.");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files",po::value<std::vector<std::string> >(&input_file_names));

  po::positional_options_description p;
  p.add("input-files",-1);

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <filenames> [options] ...\n\n";
  usage << general_options << std::endl;

  if ( vm.count("help") ) {
    std::cout << general_options << std::endl;
    return 1;
  } else if ( input_file_names.size() < 1 ) {
    std::cout << "Must specify at least one input file!" << std::endl;
    std::cout << usage.str();
    return 1;
  }

  // Iterating through all the available files
  for (unsigned file_index = 0; file_index < input_file_names.size(); file_index++) {
    std::string input_file_name = input_file_names[file_index];

    std::vector<InterestPoint> ip;
    DiskImageView<PixelGray<uint8> > src_image( input_file_name );
    ImageViewRef<PixelGray<uint8> > s_src = resample(src_image,1/scalar);
    try {
      ip = read_binary_ip_file( prefix_from_filename(input_file_name) + ".vwip" );
    } catch( Exception& e ) {
      std::cerr << "Error: " << e.what() << std::endl;
      std::cerr << usage.str() << std::endl;
      return 1;
    }

    // rgb
    ImageView<PixelRGB<uint8> > oimage = pixel_cast<PixelRGB<uint8> >( s_src*0.5 );

    // Creating the ouput file name;
    std::ostringstream output;
    output << output_prefix << prefix_from_filename(input_file_name) << ".png";

    vw_out() << "\t > Gathering statistics:\n";
    float min = 1e30, max = -1e30;
    for (std::vector<InterestPoint>::const_iterator point = ip.begin();
          point != ip.end(); ++point ) {
      if ( point->interest > max )
        max = point->interest;
      if ( point->interest < min )
        min = point->interest;
    }
    float diff = max - min;

    vw_out() << "\t > Drawing raster:\n";
    for ( std::vector<InterestPoint>::const_iterator point = ip.begin();
          point != ip.end(); ++point ) {
      float norm_i = (point->interest-min)/diff;
      PixelRGB<uint8> color(0,0,0);
      if ( norm_i < .5 ) {
        // Form of red
        color.r() = 255;
        color.g() = (unsigned char)(2*norm_i*255);
      } else {
        // Form of green
        color.g() = 255;
        color.r() = 255 - (unsigned char)(2*(norm_i-.5)*255);
      }

      // Marking dot
      Vector2i loc = Vector2i(point->ix/scalar,
                              point->iy/scalar);
      oimage(loc.x(),loc.y()) = color;

      float scale = 2*point->scale/scalar;

      // Circling point
      for (float a = 0; a < 6; a+=.392 ) {
        float a_d = a + .392;
        Vector2i start( int(scale*cos(a)),
                        int(scale*sin(a)) );
        Vector2i end( int(scale*cos(a_d)),
                      int(scale*sin(a_d)) );
        start += loc;
        end += loc;
        draw_line( oimage, color, start, end );
      }

      // Marking direction
      Vector2 dir = Vector2( cos(point->orientation),
                             sin(point->orientation) );
      draw_line( oimage, color, loc, scale*dir+loc );
    }

    std::cout << "Writing: " << output.str() << "\n";
    write_image( output.str(), oimage );

    //DiskImageResource *rsrc = DiskImageResource::create( output.str(),
    //                                                     oimage.format() );
    //block_write_image( *rsrc, oimage,
    //                   TerminalProgressCallback(InfoMessage, "\t : "));
  }
}
