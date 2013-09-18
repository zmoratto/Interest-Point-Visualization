//boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

//standard
#include <iostream>

//VisionWorkbench
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Mosaic.h>
#include <vw/InterestPoint.h>
#include <vw/InterestPoint/InterestData.h>

using namespace vw;
using namespace vw::ip;

  class BresenhamLine {
    vw::int32 x0, y0, x1, y1;
    vw::int32 x, y;
    bool steep;
    vw::int32 deltax, deltay, error, ystep;

    void setup() {
      steep = abs(y1-y0) > abs(x1-x0);
      if (steep) {
        std::swap(x0,y0);
        std::swap(x1,y1);
      }
      if ( x0 > x1 ) {
        std::swap(x0,x1);
        std::swap(y0,y1);
      }
      deltax = x1 - x0;
      deltay = abs(y1-y0);
      error = deltax / 2;
      ystep = y0 < y1 ? 1 : -1;
      x = x0; y = y0;
    }

  public:
    BresenhamLine( vw::Vector2i const& start, vw::Vector2i const& stop ) :
      x0(start[0]), y0(start[1]), x1(stop[0]), y1(stop[1]) {
      setup();
    }

    BresenhamLine( vw::int32 mx0, vw::int32 my0, vw::int32 mx1, vw::int32 my1 ) :
      x0(mx0), y0(my0), x1(mx1), y1(my1) {
      setup();
    }

    vw::Vector2i operator*() const {
      if (steep)
        return vw::Vector2i(y,x);
      else
        return vw::Vector2i(x,y);
    }

    void operator++() {
      x++;
      error -= deltay;
      if ( error < 0 ) {
        y += ystep;
        error += deltax;
      }
    }

    bool is_good() const { return x < x1; }
  };

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

void draw_line( ImageView<PixelRGB<uint8> >& image, Vector2i const& start,
                Vector2i const& end, PixelRGB<uint8> color ) {
  BBox2i box = bounding_box( image );
  BresenhamLine line( start.x(), start.y(), end.x(), end.y() );
  while (line.is_good()) {
    Vector2i pt = *line;
    if (box.contains( pt ))
      image( pt.x(), pt.y() ) = color;
    ++line;
  }
}

int main( int argc, char *argv[] ) {
  std::vector<std::string> input_file_names;
  std::string output, match_file;
  int scalar;

  po::options_description general_options("Options");
  general_options.add_options()
    ("reduce,r", po::value(&scalar)->default_value(1), "Reduce scale")
    ("output-prefix,o",po::value(&output)->default_value("matched.png"), "Output prefix")
    ("match-file", po::value(&match_file), "Define if you want to draw a specific match file.")
    ("jpeg", "Output in JPEG format.")
    ("help,h","Show this");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_file_names));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <image> <image> <image> [options] ...\n\n";
  usage << general_options << std::endl;

  if ( vm.count("help") || !input_file_names.size() ) {
    std::cout << general_options << std::endl;
    return 1;
  }

  std::cout << "Using scale: " << scalar << std::endl;

  for ( size_t i = 0; i < input_file_names.size(); ++i ) {
    for ( size_t j = i+1; j < input_file_names.size(); ++j ) {

      std::string output_filename =
        prefix_from_filename(input_file_names[i]) + "__" +
        prefix_from_filename(input_file_names[j]) + ".match";

      if ( !match_file.empty() )
        output_filename = match_file;

      if ( fs::exists( output_filename ) ) {
        // Loading up points
        std::vector<InterestPoint> ip1, ip2;
        try {
          read_binary_match_file(output_filename, ip1, ip2);
          std::cout << "\t" << output_filename << "    " << ip1.size() << " matches.\n";
        } catch( vw::Exception& e ) {
          std::cerr << "Error: " << e.what() << std::endl;
          std::cerr << usage.str() << std::endl;
          return 1;
        }

        DiskImageView<PixelRGB<float> > src1(input_file_names[i]);
        DiskImageView<PixelRGB<float> > src2(input_file_names[j]);
        ImageViewRef<PixelRGB<float> > s_src1( clamp(PixelRGB<float>(1,0,0) * subsample( src1, scalar ), 1 ) ),
          s_src2(clamp(PixelRGB<float>(0,1,1) * crop( edge_extend(subsample( src2, scalar)), bounding_box( s_src1 ) ), 1) );
        std::cout << "Compositing:\n";
        ImageView<PixelRGB<uint8> > screen_composite =
          channel_cast_rescale<uint8>(clamp((1 - 2 * (1 - s_src1) * (1 - s_src2)) + 0.5,1));

        std::cout << "Drawing lines:\n";

        for ( size_t k = 0; k < ip1.size(); k++ ) {

          Vector2i s_ip1( ip1[k].x/scalar, ip1[k].y/scalar );
          Vector2i s_ip2( ip2[k].x/scalar, ip2[k].y/scalar );

          // Draw Diamond for first image
          draw_line( screen_composite,
                     s_ip1 + Vector2i(0,5),
                     s_ip1 + Vector2i(5,0),
                     PixelRGB<uint8>(0,255,255) );
          draw_line( screen_composite,
                     s_ip1 + Vector2i(5,0),
                     s_ip1 + Vector2i(0,-5),
                     PixelRGB<uint8>(0,255,255) );
          draw_line( screen_composite,
                     s_ip1 + Vector2i(0,-5),
                     s_ip1 + Vector2i(-5,0),
                     PixelRGB<uint8>(0,255,255) );
          draw_line( screen_composite,
                     s_ip1 + Vector2i(-5,0),
                     s_ip1 + Vector2i(0,5),
                     PixelRGB<uint8>(0,255,255) );

          // Draw X for second image
          draw_line( screen_composite,
                     s_ip2 + Vector2i(-5,-5),
                     s_ip2 + Vector2i(5,5),
                     PixelRGB<uint8>(255,0,0) );
          draw_line( screen_composite,
                     s_ip2 + Vector2i(-5,5),
                     s_ip2 + Vector2i(5,-5),
                     PixelRGB<uint8>(255,0,0) );

          // Draw Black Line
          draw_line( screen_composite, s_ip1, s_ip2,
                     PixelRGB<uint8>(0,0,0) );
        }

        std::string output_image =
          prefix_from_filename(input_file_names[i]) + "__" +
          prefix_from_filename(input_file_names[j]) + ".";
        if (vm.count("jpeg")) {
          write_image( output_image+"jpg", screen_composite );
        } else {
          write_image( output_image+"png", screen_composite );
        }
      }
    }
  }
}
