#include "ORBVocabulary.h"

using namespace std;

bool load_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  bool res = voc->loadFromTextFile(infile);
  printf("Loading fom text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return res;
}

void load_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->load(infile);
  printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void load_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->save(outfile);
  printf("Saving as xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToTextFile(outfile);
  printf("Saving as text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToBinaryFile(outfile);
  printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}


int main(int argc, char **argv) {
  cout << "BoW load/save benchmark" << endl;
  ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();

  if (argc < 3)
  {
      std::cerr << "Usage: bin_vocabulary input_text_vocab_file output_binary_vocab_file" << std::endl;
      return -1;
  }

  std::string text_file_name = argv[1];
  std::string bin_file_name = argv[2];
  load_as_xml(voc, text_file_name);
  save_as_binary(voc, bin_file_name);

  return 0;
}
