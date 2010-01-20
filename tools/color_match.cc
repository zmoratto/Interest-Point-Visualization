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

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

int main( int argc, char *argv[] ) {

  std::vector<std::string> input_file_names;
  std::string output;
  int scalar;

  po::options_description general_options("Options");
  general_options.add_options()
    ("reduce,r", po::value<int>(&scalar)->default_value(1), "Reduce scale")
    ("output-prefix,o",po::value<std::string>(&output)->default_value("matched.png"), "Output prefix");

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

  if ( vm.count("help") ) {
    std::cout << general_options << std::endl;
    return 1;
  }

  std::cout << "Using scale: " << scalar << std::endl;

  for ( uint i = 0; i < input_file_names.size(); ++i ) {
    for ( uint j = i+1; j < input_file_names.size(); ++j ) {

      std::string output_filename =
        prefix_from_filename(input_file_names[i]) + "__" +
        prefix_from_filename(input_file_names[j]) + ".match";

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

        DiskImageView<PixelRGB<uint8> > src1(input_file_names[i]);
        DiskImageView<PixelRGB<uint8> > src2(input_file_names[j]);
        ImageViewRef<PixelRGB<uint8> > s_src1 = subsample(src1.impl(),scalar);
        ImageViewRef<PixelRGB<uint8> > s_src2 = subsample(src2.impl(),scalar);
        
        std::cout << "Compositing:\n";

        mosaic::ImageComposite<PixelRGB<uint8> > composite;
        composite.insert(s_src1,0,0);
        composite.insert(s_src2,s_src1.impl().cols(),0);
        composite.set_draft_mode( true );
        composite.prepare();

        // Rasterize
        ImageView<PixelRGB<uint8> > comp = composite;

        std::cout << "Drawing lines:\n";

        // Draw Red Line
        for ( unsigned int k = 0; k < ip1.size(); ++k ) {
          Vector2 start( ip1[k].x/scalar, ip1[k].y/scalar );
          Vector2 end( ip2[k].x/scalar+s_src1.impl().cols(), ip2[k].y/scalar );
          for ( float r=0; r<1.0; r+=1/norm_2(end-start) ) {
            int di = (int)(0.5 + start.x() + r*(end.x()-start.x()));
            int dj = (int)(0.5 + start.y() + r*(end.y()-start.y()));
            if (di >=0 && dj >=0 && di < comp.cols() && dj < comp.rows())
              comp(di,dj) = PixelRGB<uint8>(255, 0, 0);
          }
        }


        std::string output_image =
          prefix_from_filename(input_file_names[i]) + "__" +
          prefix_from_filename(input_file_names[j]) + ".png";
        write_image( output_image, comp );
      }
    }
  }
}
