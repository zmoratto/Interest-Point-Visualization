#include <iostream>
#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math.h>
#include <vw/Camera.h>
#include <vw/InterestPoint.h>

//boost
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::math;
using namespace vw::ip;

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

void draw_smooth_line( ImageView<PixelRGB<uint8> > const& image,
                       Vector2 const& start, Vector2 const& end,
                       PixelRGB<uint8> const& color ) {
  for (float r=0; r<1.0; r+=1/norm_2(end-start)){
    int i = (int)(0.5 + start.x() + r*(end.x()-start.x()));
    int j = (int)(0.5 + start.y() + r*(end.y()-start.y()));
    if (i >=0 && j >=0 && i < image.cols() && j < image.rows())
      image(i,j) = color;
  }
}

// MAIN
//////////////////////////////////////////////
int main( int argc, char *argv[] ) {

  // This will load up a match file .. pull out 7 nice ones .. and draw epipolar lines

  std::vector<std::string> input_file_names;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help", "brings up this" );

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

  if ( vm.count("help") || input_file_names.size() < 2 ) {
    std::cout << general_options << std::endl;
    return 1;
  }

  for ( uint i = 0; i < input_file_names.size(); ++i ) {
    for ( uint j = i+1; j < input_file_names.size(); ++j ) {

      std::string output_filename =
        prefix_from_filename(input_file_names[i]) + "__" +
        prefix_from_filename(input_file_names[j]) + ".match";

      if ( fs::exists( output_filename ) ) {
        // Loading up points
        std::vector<InterestPoint> ip1, ip2;
        try {
          ip::read_binary_match_file(output_filename, ip1, ip2);
          std::cout << "\t" << output_filename << "    " << ip1.size() << " matches.\n";
        } catch( vw::Exception& e ) {
          std::cerr << "Error: " << e.what() << std::endl;
          std::cerr << usage.str() << std::endl;
          return 1;
        }

        // Selecting 7
        std::vector<Vector3> ipl, ipr;
        for ( uint k = 0; k < 7; k++ ) {
          uint temp = float(ip1.size()-1)*float(k)/6.0;
          ipl.push_back(Vector3(ip1[temp].ix,ip1[temp].iy,1));
          ipr.push_back(Vector3(ip2[temp].ix,ip2[temp].iy,1));
        }

        // Solving for fundamental matrix
        Fundamental7FittingFunctor func;
        try {
          Matrix<double> fundie = func(ipl,ipr);
        } catch( vw::Exception& e ) {
          std::cout << "Errored out: " << e.what() << "\n";
          std::cout << "--- used input ---\n";
          for ( uint i = 0; i < ipl.size(); i++ ) {
            std::cout << "\t" << ipl[i] << " + " << ipr[i] << "\n";
          }
          return 1;
        }
        std::cout << "Num sol: " << func.num_solutions() << "\n";

        // Loading images
        DiskImageView<PixelRGB<uint8> > src1(input_file_names[i]);
        DiskImageView<PixelRGB<uint8> > src2(input_file_names[j]);
        ImageView<PixelRGB<uint8> > new1 = src1;
        ImageView<PixelRGB<uint8> > new2 = src2;

        // Draw RGB lines for each solution
        for ( uint sol = 0; sol < func.num_solutions(); sol++ ) {

          Matrix<double> fundie = func.fundamental_matrix(sol);
          std::cout << "Fundie: " << fundie << "\n";

          PixelRGB<uint8> color;
          switch(sol) {
          case 0:
            color = PixelRGB<uint8>(255,0,0);
            break;
          case 1:
            color = PixelRGB<uint8>(0,255,0);
            break;
          case 2:
          default:
            color = PixelRGB<uint8>(0,0,255);
            break;
          }

          for ( uint k = 0; k < ipr.size(); k++ ) {
            Vector3 line = transpose(fundie)*ipr[k];
            Vector2 start(0, line.z()/(-line.y()) );
            Vector2 end(new1.cols(), (line.x()*float(new2.cols())+line.z())/(-line.y()) );
            draw_smooth_line( new1, start, end, color );
            new1(ipl[k].x(),ipl[k].y()) = PixelRGB<uint8>(255,255,255);
          }

          // Lines are in eq ax+by+c = 0; y = (ax + c) / -b
          for ( uint k = 0; k < ipl.size(); k++ ) {
            Vector3 line = fundie*ipl[k];
            Vector2 start(0, line.z()/(-line.y()) );
            Vector2 end(new2.cols(), (line.x()*float(new2.cols())+line.z())/(-line.y()) );
            draw_smooth_line( new2, start, end, color );
            new2(ipr[k].x(),ipr[k].y()) = PixelRGB<uint8>(255,255,255);
          }
        }

        write_image("output_left.jpg", new1);
        write_image("output_right.jpg", new2);
      }

    }
  }

  return 0;
}
