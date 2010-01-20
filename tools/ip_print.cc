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

int main( int argc, char *argv[] ){

  std::vector<std::string> input_file_names;

  po::options_description general_options("Options");
  general_options.add_options()
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
  usage << "Usage: " << argv[0] << " <filenames (match files or vwips)> [options] ...\n\n";
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

    if ( boost::iends_with(input_file_name,".vwip") ) {
      std::vector<InterestPoint> ip;
      ip = read_binary_ip_file(input_file_name);
      vw_out() << "VWIP file: \"" << input_file_name << "\"\n";
      for ( uint i = 0; i < ip.size(); i++ )
        vw_out() << "\t(" << ip[i].x << ", " << ip[i].y << ")\n";
    } else if ( boost::iends_with(input_file_name,".match") ) {
      std::vector<InterestPoint> ip1, ip2;
      read_binary_match_file(input_file_name,ip1,ip2);
      vw_out() << "MATCH file: \"" << input_file_name << "\"\n";
      for ( uint i = 0; i < ip1.size(); i++ )
        vw_out() << "\t(" << ip1[i].x << ", " << ip1[i].y << ")-("
                 << ip2[i].x << ", " << ip2[i].y << ")\n";
    }
    vw_out() << "\n";
  }
}
