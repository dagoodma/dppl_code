#ifndef _DPP_ALGORITHM_NEARESTNEIGHBORDTSP_H_
#define _DPP_ALGORITHM_NEARESTNEIGHBORDTSP_H_

namespace DPP {

/*
 * Alternating algorithm for the DTSP.
 */
class NearestNeighborDTSP : public AlgorithmDTSP {
public:
    enum AlgorithmName = {NEAREST_NEIGHBOR, ALTERNATING, RANDOMIZED};

    NearestNeighborDTSP(void) {
        m_name = AlgorithmName::NEAREST_NEIGHBOR;
    }
};

} // namespace DPP
#endif // _DPP_ALGORITHM_NEARESTNEIGHBORDTSP_H_