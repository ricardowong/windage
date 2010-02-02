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

// Author: Friedrich Fraundorfer, fraundorfer@inf.ethz.ch

#include "ff_voctree.h"
#include <wchar.h>
#include "iostream"
#include "math.h"

using namespace std;


ff_voctree::ff_voctree(void) {
	m_init = 0;
}

ff_voctree::~ff_voctree(void) {
	if (m_init == 1) {
		clean();
		m_init = 0;
	}
}


int ff_voctree::init(char *fname, int truncate) {

	// load voctree
	FILE *fin;
	fin = fopen(fname, "rb");
	if(fin ==0) return 0;
	fread(&m_visualwords, sizeof(unsigned int),1,fin);
	fread(&m_levels, sizeof(unsigned int),1,fin);
	fread(&m_splits, sizeof(unsigned int),1,fin);
	fread(&m_nrcenters, sizeof(unsigned int),1,fin);
	if(m_levels > 10 || m_splits > 100000)
	{
		m_visualwords = 0;
		m_levels = 0;
		m_splits = 0;
		m_nrcenters = 0;
		fclose(fin);
		return 0;
	}
	if(truncate)
	{
		m_levels--;
		int  nc = m_nrcenters - m_visualwords;
		m_voc = new unsigned char[nc*128];
		fread(m_voc, sizeof(unsigned char),nc*128,fin);
		fseek(fin, m_visualwords*128, SEEK_CUR);
		m_cellinfo = new unsigned char[nc];
		fread(m_cellinfo, sizeof(unsigned char),nc,fin);
		m_nrcenters = nc;
		m_visualwords/=m_splits;
	}else
	{
		m_voc = new unsigned char[m_nrcenters*128];
		fread(m_voc, sizeof(unsigned char),m_nrcenters*128,fin);
		m_cellinfo = new unsigned char[m_nrcenters];
		fread(m_cellinfo, sizeof(unsigned char),m_nrcenters,fin);
	}
	fclose(fin);

	m_vwtable = new unsigned int[m_nrcenters];

	int step;
	int abspos = 0;
	// create vwtable
	for (unsigned int level = 0; level < m_levels; level++) {
		step = (int)( m_visualwords/pow((double)m_splits,(double)level+1));
		for (int i = 0; i < pow((double)m_splits,(double)level+1); i++) {
			m_vwtable[abspos] = step*i;
			abspos++;
		}
	}

	m_init = 1;
	return 1;
}

void ff_voctree::clean(void) {
	if (m_init == 1) {
		delete [] m_voc;
		delete [] m_cellinfo;
		delete [] m_vwtable;
		m_init = 0;
	}
}

void ff_voctree::quantize(unsigned int *vwi, unsigned char *sift	  ) {

	m_centerpos = 0;
	m_previndex = 0;
	m_offset = -1;
	for (unsigned int level = 0; level < m_levels; level++)
	{

		m_offset = (m_offset + 1)*m_splits;



		m_centerpos = (m_centerpos+m_previndex)*m_splits;

		//mindist = 256*5000; // L1
		m_mindist = 256*256*5000; // L2

		unsigned char * voc = m_voc + (m_centerpos+m_offset)*128;
		for (unsigned int i = 0; i < m_splits; i++)
		{
			m_dist = 0;
			unsigned char * ps = sift;
			for (int k = 0; k < 128; k++, ps++, voc++)
			{
				int diff = ps[0]-voc[0];
				//dist += abs(diff));    //L1
				m_dist += (diff*diff);    //L2
			}
			if (m_dist < m_mindist)
			{
				m_mindist = m_dist;
				m_minindex = i;
			}
		}

		m_previndex = m_minindex;

		if (m_cellinfo[m_centerpos+m_offset] > 0) {
			*vwi = m_vwtable[m_centerpos+m_offset+m_previndex];
			return;
		}
	}

*vwi = m_vwtable[m_centerpos+m_offset+m_previndex];
}

int ff_voctree::calcnrcenters(int splits, int levels) {
	return (int)(pow((double)splits,(double)levels+1)-1)/(splits-1)-1;
}
