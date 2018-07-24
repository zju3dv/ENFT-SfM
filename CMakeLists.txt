cmake_minimum_required(VERSION 3.2)
project(ENFT)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse -msse2 -msse3 -msse4 -msse4a -std=c++11 -fpermissive -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4a -std=c++11 -fpermissive -w")
set(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS} -O0 -g -ggdb")

set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")
if(CMAKE_BUILD_TYPE MATCHES Debug)
	add_definitions(-D_DEBUG)
endif()

include_directories(		
		${PROJECT_SOURCE_DIR}
		${PROJECT_SOURCE_DIR}/3rdparty/levmar
		${PROJECT_SOURCE_DIR}/3rdparty
		${PROJECT_SOURCE_DIR}/ENFT
		${PROJECT_SOURCE_DIR}/Localizer
		${PROJECT_SOURCE_DIR}/RegisterGPS
)

# tmp modify for glsl debug
include_directories(
		${PROJECT_SOURCE_DIR}/3rdparty/
		${PROJECT_SOURCE_DIR}/3rdparty/levmar/include
		${PROJECT_SOURCE_DIR}/3rdparty/clapack/include
		${PROJECT_SOURCE_DIR}/3rdparty/CVD/include
)

set(
		ENFT_SOURCE_FILES
		ENFT/ProgramGL/ProgramGL.cpp
		ENFT/Utility/Polynomial.cpp
		ENFT/FeatureTracking/FeatureTracker.cpp
		ENFT/FeatureTracking/FeatureTrackerIO.cpp
		ENFT/FeatureTracking/FeatureTrackerEnft.cpp
		ENFT/FeatureTracking/FeatureTrackerSift.cpp
		ENFT/FeatureTracking/FeatureExtractorSift.cpp
		ENFT/FeatureTracking/FeatureTrackerFlowVoting.cpp
		ENFT/TrackMatching/TrackMatcher.cpp
		ENFT/TrackMatching/TrackMatcherIO.cpp
		ENFT/TrackMatching/MatchingMatrix.cpp
		ENFT/TrackMatching/TrackMatcherEnft.cpp
		ENFT/TrackMatching/TrackMatcherClustering.cpp
		ENFT/TrackMatching/TrackMatchingFeatureMatching.cpp
		ENFT/LinearAlgebra/Matrix3.cpp
		ENFT/LinearAlgebra/Matrix4.cpp
		ENFT/LinearAlgebra/Matrix5.cpp
		ENFT/LinearAlgebra/Matrix6.cpp
		ENFT/LinearAlgebra/Matrix7.cpp
		ENFT/LinearAlgebra/Matrix8.cpp
		ENFT/LinearAlgebra/Matrix11.cpp
		ENFT/LinearAlgebra/MatrixMxN.cpp
		ENFT/SfM/CameraEPnP.cpp
		ENFT/SfM/Homography.cpp
		ENFT/SfM/PlaneSolver.cpp
		ENFT/SfM/Point3DSolver.cpp
		ENFT/SfM/EssentialMatrix.cpp
		ENFT/SfM/CameraArrayEPnP.cpp
		ENFT/SfM/ProjectiveMatrix.cpp
		ENFT/SfM/HomographySolver.cpp
		ENFT/SfM/FundamentalMatrix.cpp
		ENFT/SfM/TranslationSolver.cpp
		ENFT/SfM/ProjectiveMatrixSolver.cpp
		ENFT/SfM/AbsoluteQuadricEstimator.cpp
		ENFT/SfM/RelativeTranslationSolver.cpp
		ENFT/SfM/RigidTransformationSolver.cpp
		ENFT/SfM/RotationTransformationSolver.cpp
		ENFT/SfM/SimilarityTransformationSolver.cpp
		ENFT/SfM/EssentialMatrixFivePointAlgorithm.cpp
		ENFT/SfM/FundamentalMatrixEightPointAlgorithm.cpp
		ENFT/SequenceSet/SequenceSet.cpp
		ENFT/SequenceSet/SequenceSetDataAssociation.cpp
		ENFT/SequenceSet/SequenceSetFrame.cpp
		ENFT/SequenceSet/SequenceSetIO.cpp
		ENFT/SequenceSet/SequenceSetMeasurement.cpp
		ENFT/SequenceSet/SequenceSetTrack.cpp
		ENFT/Sequence/Sequence.cpp
		ENFT/Sequence/SequenceIO.cpp
		ENFT/Sequence/SequenceTag.cpp
		ENFT/Sequence/SequenceFrame.cpp
		ENFT/Sequence/SequenceDepth.cpp
		ENFT/Sequence/SequenceTrack.cpp
		ENFT/Sequence/SequenceSynthesis.cpp
		ENFT/Sequence/SequenceMeasurement.cpp
		ENFT/Sequence/SequenceTransformation.cpp
		ENFT/Sequence/SequenceDataAssociation.cpp
		ENFT/Sequence/SequenceBundleAdjustment.cpp
		ENFT/Sequence/SequenceBundleAdjustorData.cpp
		ENFT/Sequence/SequenceBundleAdjustorDataDepth.cpp
		ENFT/Sequence/SequenceBundleAdjustorDataProjective.cpp
		ENFT/Sequence/SequenceBundleAdjustorDataIntrinsicVariable.cpp
		ENFT/Viewing/ViewerSequence.cpp
		ENFT/Viewing/ViewerSequenceSet.cpp
		ENFT/Viewing/ViewerSequenceDepth.cpp
		ENFT/CameraTracking/CameraTracker.cpp
		ENFT/CameraTracking/CameraTrackerSfM.cpp
		ENFT/CameraTracking/CameraTrackerIO.cpp
		ENFT/CameraTracking/CameraTrackerDepth.cpp
		ENFT/CameraTracking/CameraTrackerDepthIO.cpp
		ENFT/CameraTracking/CameraTrackerDepthSfM.cpp
		ENFT/SequenceSet/SequenceSetFrame.cpp
		ENFT/SequenceSet/SequenceSetBundleAdjustment.cpp
		ENFT/SequenceSet/SequenceSetBundleAdjustorData3DSimilarity.cpp
		ENFT/SequenceSet/SegmentSetBundleAdjustorData2DSimilarity.cpp
		ENFT/SequenceSet/SequenceSetTrack.cpp
		ENFT/SequenceSet/SequenceSetMeasurement.cpp
		ENFT/SequenceSet/SequenceSet.cpp
		ENFT/SequenceSet/SequenceSetDataAssociation.cpp
		ENFT/SequenceSet/SequenceSetIO.cpp
		ENFT/SequenceSet/SequenceTransformationOptimizerDataSimilarity.cpp
		ENFT/SequenceSet/SequenceTransformationOptimizerDataScale.cpp
		ENFT/SequenceSet/SequenceSetSynthesis.cpp
		ENFT/SequenceRegistration/SequenceRegisterorIO.cpp
		ENFT/SequenceRegistration/SequenceRegisterorSfM.cpp
		ENFT/SequenceRegistration/SequenceRegisteror.cpp
		ENFT/SequenceRegistration/SequenceRegisterorSegmentation.cpp
)

### ENFT.lib
add_library(${PROJECT_NAME} STATIC ${ENFT_SOURCE_FILES})

target_link_libraries(
		${PROJECT_NAME}
		${PROJECT_SOURCE_DIR}/3rdparty/levmar/linux_lib/liblevmar.a
		${PROJECT_SOURCE_DIR}/3rdparty/clapack/linux_lib/liblapack.a
		${PROJECT_SOURCE_DIR}/3rdparty/clapack/linux_lib/libblas.a
		${PROJECT_SOURCE_DIR}/3rdparty/clapack/linux_lib/libf2c.a
		${PROJECT_SOURCE_DIR}/3rdparty/CVD/linux_lib/libcvd.a
		-lX11
		-lGL
		-lGLU
		-lGLEW
		-lglut
        -ljpeg
        -lpng
        -ltiff
)

### ENFT exec
add_executable(runENFT ENFT/ENFT.cpp)
target_link_libraries(runENFT ${PROJECT_NAME})
