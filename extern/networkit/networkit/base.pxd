from libcpp cimport bool as bool_t

cdef extern from "<networkit/base/Algorithm.hpp>" namespace "NetworKit":
	cdef cppclass _Algorithm "NetworKit::Algorithm":
		_Algorithm()
		void run() nogil except +
		bool_t hasFinished() except +

cdef class Algorithm:
	cdef _Algorithm *_this

# This creates a cyclic import and is currently unused
#from .dynamics cimport _GraphEvent, GraphEvent

#cdef extern from "<networkit/base/DynAlgorithm.hpp>":
#
#	cdef cppclass _DynAlgorithm "NetworKit::DynAlgorithm":
#		void update(_GraphEvent) except +
#		void updateBatch(vector[_GraphEvent]) except +
