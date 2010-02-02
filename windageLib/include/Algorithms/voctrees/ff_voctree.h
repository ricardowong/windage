/* Copyright (c) 2008, Eidgenössische Technische Hochschule Zürich, ETHZ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Eidgenössische Technische Hochschule Zürich, ETHZ "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL the Eidgenössische Technische Hochschule Zürich, ETHZ BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/////////////////////////////////////////////////////////
// Vocabulary tree class
// Class reads a vocabulary tree from file and provides methods
// to quantize SIFT descriptors with the vocabulary tree
// To compute a vocabulary tree from SIFT vectors use the Matlab script voctree.m
/////////////////////////////////////////////////////////

// Author: Friedrich Fraundorfer, fraundorfer@inf.ethz.ch

#ifndef FF_VOCTREE
#define FF_VOCTREE


class ff_voctree {
public:
	ff_voctree(void);
	~ff_voctree(void);
	
	// Class initialization, memory allocation and voctree read from file
	// Param: fname ... (IN) path to voctree
	int init(char *fname, int truncate = 0);

	// Memory deallocation
	void clean(void);

	// Quantize a 128 dimensional SIFT vector using the voctree. Distances are computed in L2 norm.
	// Param: vwi ... (OUT) visual word for the SIFT vector
	//        sift... (IN) input SIFT vector 128 chars normalized to 0..255
	void quantize(unsigned int *vwi, unsigned char *sift		);
	// Computes the number of treenodes (clustercenters) for given split and level
	int calcnrcenters(int splits, int levels);

	// Returns number of levels for currently loaded voctree
	inline int nrlevels(void) { return m_levels; };
	// Returns number of splits for currently loaded voctree
	inline int nrsplits(void) { return m_splits; };
	// Returns number of centers for currently loaded voctree
	inline int nrcenters(void) { return m_nrcenters; };
	// Returns number of vw's for current loaded voctree
	inline int nrvisualwords(void) { return m_visualwords; };

	///
	inline int isvalid(void) {return m_init;}
private:
	int m_init;
	unsigned char *m_voc;
	unsigned char *m_cellinfo;
	unsigned int *m_vwtable;
	unsigned int m_levels;
	unsigned int m_splits;
	unsigned int m_nrcenters;
	unsigned int m_visualwords;
	int   m_half; 
	__int64 m_centerpos;
	double m_mindist;
	int m_minindex;
	double m_dist;
	int m_previndex;
	int m_offset;

};

#endif //FF_VOCTREE