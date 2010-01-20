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

  std::string match_file_name;
  std::string input1_file_name, input2_file_name;
  std::string output_prefix;

  po::options_description general_options("Options");
  general_options.add_options()
    ("input1,i1", po::value<std::string>(&input1_file_name), "Input file 1")
    ("input2,i2", po::value<std::string>(&input2_file_name), "Input file 2")
    ("match,m", po::value<std::string>(&match_file_name), "Match file")
    ("output-prefix,o",po::value<std::string>(&output_prefix)->default_value("color_pts_"), "Output prefix")
    ("help,h", "Help");

  po::positional_options_description p;
  p.add("input1",1);
  p.add("input2",1);
  p.add("match",1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <input1-file> <input2-file> <match-file> [options] ...\n\n";
  usage << general_options << std::endl;

  if ( vm.count("help") ) {
    std::cout << general_options << std::endl;
    return 1;
  } else if ( (vm.count("input1") != 1) &&
	      (vm.count("input2") != 1) &&
	      (vm.count("match") != 1) ) {
    std::cout << "You are missing an argument\n";
    std::cout << usage.str();
    return 1;
  }

  // Code Monkey!
  std::vector<InterestPoint> ip1, ip2;
  DiskImageView<PixelRGB<uint8> > dsk_image1( input1_file_name );
  DiskImageView<PixelRGB<uint8> > dsk_image2( input2_file_name );
  try {
    read_binary_match_file(match_file_name, ip1, ip2);
    std::cout << "\t" << match_file_name << "    " << ip1.size() << " matches.\n";
  } catch( vw::Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << usage.str() << std::endl;
    return 1;
  }

  // Creating the ouput file name;
  std::ostringstream output1, output2;
  output1 << output_prefix << input1_file_name;
  output2 << output_prefix << input2_file_name;

  // Rasterize points
  vw::ImageView<vw::PixelRGB<vw::uint8> > oimage1 = 
    0.5*pixel_cast<PixelGray<uint8> >( channel_cast_rescale<uint8>( dsk_image1.impl() ) );
  vw::ImageView<vw::PixelRGB<vw::uint8> > oimage2 =
    0.5*pixel_cast<PixelGray<uint8> >( channel_cast_rescale<uint8>( dsk_image2.impl() ) );

  for (unsigned i = 0; i < ip1.size(); ++i){
    PixelHSV<uint8> hsv_color(255*float(i)/float(ip1.size()), 255, 255);
    PixelRGB<uint8> write_color(hsv_color);

    oimage1(ip1[i].x,ip1[i].y) = write_color;
    oimage2(ip2[i].x,ip2[i].y) = write_color;

    // Make circle in second image
    for ( float a = 0; a < 6; a += .392 ) {
      float a_d = a + .392;
      Vector2i start( int(5*cos(a)+ip2[i].x),
                      int(5*sin(a)+ip2[i].y) );
      Vector2i end( int(5*cos(a_d)+ip2[i].x),
                    int(5*sin(a_d)+ip2[i].y) );
      Vector2i conversion = Vector2i(ip1[i].x,ip1[i].y) - Vector2i(ip2[i].x,ip2[i].y);
      draw_line( oimage2, write_color, start, end );
      draw_line( oimage1, write_color, start+conversion, end+conversion );
    }
  }

  write_image( output1.str(), oimage1 );
  write_image( output2.str(), oimage2 );
}
