## Usage

1) git submodule update --init --recursive
2) mkdir build && cd build
3) cmake .. && make

If want to build tests, then use the following instead of 3) :
cmake .. -DUSE_TEST=ON && make


### Feature Tracker App:
./feature_tracker/feature_tracker_app -p ~/Project/vio/data/desk2_subset/ -f jpg --detector FAST(SURF, ORB, ...) --descriptor DAISY(ORB, SIFT, ...) --matcher_type GRID(OCV)


##Log:

03/28/2017

Now run
./feature_tracker/feature_tracker_app -p ../feature_tracker/test/test_data/long_seq/ -f jpg --detector FAST --descriptor DAISY --matcher_type GRID
./feature_tracker/feature_tracker_app -p ../feature_tracker/test/test_data/long_seq/ -f jpg --detector FAST --descriptor DAISY --matcher_type OCV

The difference is OCV matching has more long feature tracks than grid search method. Maybe increase the search region? Although grid is much faster than OCV
TODO: Evaluate how to decide use which method.

