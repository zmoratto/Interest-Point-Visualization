// boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//standard
#include <iostream>

//VisionWorkbench
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>

using namespace vw;
using namespace vw::ip;

static void remove_duplicates( std::vector<InterestPoint> &ip1,
                               std::vector<InterestPoint> &ip2 ) {
  std::vector<InterestPoint> new_ip1, new_ip2;

  for (unsigned i = 0; i < ip1.size(); ++i ) {
    bool bad_entry = false;
    for (unsigned j=i; j < ip1.size(); ++j ) {
      if (i != j &&
          ((ip1[i].x == ip1[j].x && ip1[i].y == ip1[j].y) ||
           (ip2[i].x == ip2[j].x && ip2[i].y == ip2[j].y)))
        bad_entry = true;
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }

  ip1 = new_ip1;
  ip2 = new_ip2;
}

std::vector<BBox2i> bounding_blocks( BBox2i const& orginal,
                                     int32 block_width, int32 block_height ) {
  std::vector<BBox2i> bboxes;

  Vector2 min = orginal.min();

  int32 j_offset = 0;
  while ( j_offset < orginal.height() ) {
    int32 j_dim = ( orginal.height() - j_offset) < block_height ? ( orginal.height() - j_offset ) : block_height;
    int32 i_offset = 0;
    while ( i_offset < orginal.width() ) {
      int32 i_dim = ( orginal.width() - i_offset) < block_width ? ( orginal.width() - i_offset ) : block_width;
      bboxes.push_back( BBox2i( i_offset + min.x(),
                                j_offset + min.y(),
                                i_dim, j_dim ) );
      i_offset += i_dim;
    }
    j_offset += j_dim;
  }

  return bboxes;
}

int main( int argc, char *argv[] ){

  std::string match_file_name;
  std::string output_prefix;

  po::options_description general_options("Options");
  general_options.add_options()
    ("match,m", po::value<std::string>(&match_file_name), "Match file")
    ("output-prefix,o", po::value<std::string>(&output_prefix), "Output Prefix")
    ("help,h", "Gives you this screen");

  po::positional_options_description p;
  p.add("match",1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <match-file> [options] ... \n\n";
  usage << general_options << std::endl;

  if ( vm.count("help") ) {
    std::cout << general_options << std::endl;
    return 1;
  } else if ( vm.count("match") != 1 ) {
    std::cout << usage.str();
    return 1;
  }

  // Code Monkey!
  std::vector<InterestPoint> ip1, ip2;
  try {
    read_binary_match_file( match_file_name, ip1, ip2);
  } catch( vw::Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << usage.str() << std::endl;
    return 1;
  }

  // RANSAC
  {
    std::cout << "Performing RANSAC to throw out outliers:\n";
    std::cout << "\t> Found " << ip1.size() << " matches.\n";
    //remove_duplicates( ip1, ip2 );
    std::vector<Vector3> ransac_ip1( ip1.size() );
    std::vector<Vector3> ransac_ip2( ip2.size() );
    for ( unsigned i = 0; i < ip1.size(); ++i ) {
      ransac_ip1[i] = Vector3( ip1[i].x, ip1[i].y, 1 );
      ransac_ip2[i] = Vector3( ip2[i].x, ip2[i].y, 1 );
    }
    math::RandomSampleConsensus< math::HomographyFittingFunctor,math::InterestPointErrorMetric> ransac( math::HomographyFittingFunctor(),
                                                                                                        math::InterestPointErrorMetric(),
                                                                                            20 ); //inlier_threshold
    Matrix<double> T = ransac( ransac_ip1, ransac_ip2 );
    std::cout << "\t> T = " << T << std::endl;

    std::vector<int> indices;
    indices = ransac.inlier_indices(T, ransac_ip1, ransac_ip2 );

    std::cout << "\t> Found " << indices.size() << " acceptable matches.\n";

    std::vector<InterestPoint> final_ip1, final_ip2;
    for ( unsigned i=0; i < indices.size(); ++i ) {
      final_ip1.push_back( ip1[ indices[i] ] );
      final_ip2.push_back( ip2[ indices[i] ] );
    }
    ip1 = final_ip1;
    ip2 = final_ip2;
  }

  // Find bounding box
  std::cout << "Building Bounding Boxes:\n";
  BBox2i first_bbox;
  for (unsigned i = 0; i < ip1.size(); ++i) {
    first_bbox.grow( Vector2( ip1[i].x, ip1[i].y ) );
  }
  Vector2 min = first_bbox.min();
  std::cout << "\t> min_x = " << min.x() << std::endl;
  std::cout << "\t> min_y = " << min.y() << std::endl;
  std::cout << "\t> width = " << first_bbox.width() << std::endl;
  std::cout << "\t> height = " << first_bbox.height() << std::endl;
  std::vector<BBox2i> bboxes = bounding_blocks( first_bbox, 200, 200 );
  std::vector<float> histogram;
  histogram.resize( bboxes.size() );

  // Calculating Statistics
  std::cout << "Gathering Statistics:\n";
  for (unsigned b = 0; b < bboxes.size(); ++b ) {
    for (unsigned i = 0; i < ip1.size(); ++i ) {
      if ( bboxes[b].contains( Vector2( ip1[i].x, ip1[i].y )) )
        histogram[b] += 1;
    }
    histogram[b] = (200*200)/(bboxes[b].width()*bboxes[b].height())*histogram[b];
  }

  float mean = 0.0;
  float sum = 0.0;
  for (unsigned b = 0; b < histogram.size(); ++b ) {
    mean += histogram[b];
    if ( bboxes[b].width() > 0 && bboxes[b].height() > 0 )
      sum++;
  }
  mean /= sum;

  float std_dev = 0.0;
  for (unsigned b = 0; b < histogram.size(); ++b ) {
    if ( bboxes[b].width() > 0 && bboxes[b].height() > 0 )
      std_dev += (histogram[b] - mean)*(histogram[b] - mean);
  }
  std_dev /= (sum);
  std_dev = sqrt( std_dev );

  std::cout << "\t> mean = " << mean << std::endl;
  std::cout << "\t> std_dev = " << std_dev << std::endl;

  // If this is around, we'll write a correction
  if (vm.count("output-prefix")) {
    std::ostringstream ostr;
    ostr << output_prefix << match_file_name;

    write_binary_match_file( ostr.str(), ip1, ip2 );
  }
}
