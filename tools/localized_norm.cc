//boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//standard
#include <iostream>

//VisionWorkbench
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/InterestPoint.h>

int main( int argc, char *argv[] ) {

  int window_size;
  std::string input_file_name, output_file_name;
  
  po::options_description general_options("Options");
  general_options.add_options()
    ("window,w", po::value<int>(&window_size)->default_value(20), "Number of divisions to break the image into (actually it's this number squared is the number of divisions).")
    ("input-file", po::value<std::string>(&input_file_name), "Explicity specify the input file")
    ("output-file", po::value<std::string>(&output_file_name), "Explicity specify the output file")
    ("help,h", "Display this help message" );

  po::positional_options_description p;
  p.add("input-file", 1);
  p.add("output-file",1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv).options(general_options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <input-file> <output-file> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    std::cout << general_options << std::endl;
    return 1;
  } else if ( (vm.count("input-file") != 1) && (vm.count("output-file") != 1) ){
    std::cout << usage.str();
    return 1;
  }

  // Hey, it code! I looooove code.

  vw::ImageView<float> image;
  try {
    read_image( image, input_file_name );
  }
  catch( vw::Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "Usage: vwconvert <source> <destination>" << std::endl;
    return 1;
  }

  std::vector<vw::BBox2i> bboxes = image_blocks(image.impl(), window_size, window_size );

  for (unsigned i = 0; i < bboxes.size(); ++i) {
    vw::ImageView<float> subsection = copy(crop(image, bboxes[i]));
    //vw::ImageView<float> weight = subsection; //bad: weight really is equal to subsection
    vw::ImageView<float> weight = copy(subsection);
    fill( weight, 1.0 );
    
    std::vector<float> histogram;
    vw::ip::weighted_histogram( subsection,
				weight,
				histogram,
				0.0, 1.0, 10 );

    float mean_index = 0.0;
    float sum = 0.0;
    for (unsigned n = 0; n < histogram.size(); ++n) {
      mean_index += n*histogram[n];
      sum += histogram[n];
    }
    mean_index /= sum;  // I feel somehow, there is a better way. 

    float std_dev = 0.0;
    for (unsigned n = 0; n < histogram.size(); ++n) {
      if ( histogram[n] ) {
	std_dev += (n - mean_index)*(n - mean_index)*(histogram[n]/sum);
      }
    }
    std_dev = sqrt(std_dev);
    

    //std::cout << "Segment " << i << std::endl;
    //std::cout << "\t> mean_index = " << mean_index << std::endl;
    //std::cout << "\t> std_dev = " << std_dev << std::endl;


    float min, max;
    min = mean_index - 2*std_dev;
    if ( min < 0 )
      min = 0;
    max = mean_index + 2*std_dev;
    if ( max > (histogram.size() - 1) )
      max = histogram.size() - 1;

    //std::cout << "\t> min_index = " << min << std::endl;
    //std::cout << "\t> max_index = " << max << std::endl;

    //converting from index to value
    min = min/(histogram.size() - 1);
    max = max/(histogram.size() - 1);

    //std::cout << "\t> min = " << min << std::endl;
    //std::cout << "\t> max = " << max << std::endl;
    

    // normalizing subsection
    for (vw::ImageView<float>::iterator iter = subsection.begin(); iter < subsection.end(); ++iter){
      if ((max - min) < .1){
	*iter = max;
      } else {
	*iter = (*iter - min)/(max-min);
      }
    }

    //crop(image, bboxes[i]) = normalize(subsection,min,max);
    crop(image, bboxes[i]) = subsection;
  }

  write_image( output_file_name, image );

  return 0;
}
