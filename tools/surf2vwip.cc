// boost
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
namespace po = boost::program_options;

//standard
#include <iostream>
#include <vector>

//VisionWorkbench
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::ip;

void read_surf_file( std::string& surf_file, std::list<InterestPoint>& ip ) {

  // This is just basic rip of the SURF code
  std::ifstream ipfile( surf_file.c_str() );
  if( !ipfile ) {
    std::cerr << "ERROR in read_surf_file(): "
	      << "Couldn't open file '" << surf_file.c_str() << "'!" << std::endl;
    return;
  }
  
  // Load the file header
  unsigned vlen;
  unsigned count;
  ipfile >> vlen >> count;

  std::cout << "Found SURF file with " << count << " points.\n";

  ip.clear();

  // Load the interst points
  for ( unsigned n=0; n<count; n++ ) {
    // circular regions with diameter 5 x scale
    float x, y, a, b, c, ori;
    InterestPoint temp;

    // Read n the region data
    ipfile >> x >> y >> a >> b >> c >> ori;

    float det = sqrt((a-c)*(a-c) + 4.0*b*b);
    float e1 = 0.5*(a+c + det);
    float e2 = 0.5*(a+c - det);
    float l1 = (1.0/sqrt(e1));
    float l2 = (1.0/sqrt(e2));
    float sc = sqrt( l1*l2 );

    temp.x = x;
    temp.y = y;
    temp.scale = sc/2.5;
    temp.orientation = ori;

    temp.ix = (int)x;
    temp.iy = (int)y;
    
    // Reading in the Laplacian
    ipfile >> temp.interest;

    // Reading in the descriptor
    temp.descriptor.set_size(vlen-1);
    for (unsigned j = 0; j < vlen -1; j++)
      ipfile >> temp.descriptor[j];
    
    ip.push_back(temp);
  }
  std::cout << "IP found to have " << ip.size() << ".\n";
}

int main( int argc, char *argv[] ) {

  std::vector<std::string> input_file_names;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Gives you this screen");

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
  usage << "Usage: " << argv[0] << " <surf-files> [options] ...\n\n";
  usage << general_options << std::endl;

  if ( vm.count("help") ) {
    std::cout << general_options << std::endl;
    return 1;
  } else if ( vm.count("input-files") != 1 ) {
    std::cout << "Must specify at least one file!" << std::endl;
    std::cout << usage.str();
    return 1;
  }
  
  // Iterating through files
  for (unsigned fi = 0; fi < input_file_names.size(); fi++) {
    
    std::string input_surf = input_file_names[fi];

    std::list<InterestPoint> ip;
    try {
      read_surf_file( input_surf, ip );
    } catch( vw::Exception& e ) {
      std::cerr << "Error: " << e.what() << std::endl;
      std::cerr << usage.str() << std::endl;
      return 1;
    }

    std::cout << "IP found to have " << ip.size() << ".\n";
    
    // Hazza!
    {
      std::vector<std::string> list;
      boost::split( list, input_surf, boost::is_any_of("."));
      std::ostringstream ostr;
      ostr << list[0] << ".vwip";
      write_binary_ip_file( ostr.str(), ip );
    }
  }
}
