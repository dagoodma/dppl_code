#ifndef _DPP_ALGORITHM_ALTERNATINGDTSP_H_
#define _DPP_ALGORITHM_ALTERNATINGDTSP_H_

namespace DPP {

/*
 * Alternating algorithm for the DTSP.
 */
class AlternatingDTSP : public AlgorithmDTSP {
public:
    enum AlgorithmName = {NEAREST_NEIGHBOR, ALTERNATING, RANDOMIZED};

    AlternatingDTSP(void) {
        m_name = AlgorithmName::ALTERNATING;
    }
};

} // namespace DPP
#endif // _DPP_ALGORITHM_ALTERNATINGDTSP_H_