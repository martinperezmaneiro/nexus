#ifndef FULLRINGINF_
#define FULLRINGINF_

#include "BaseGeometry.h"

class G4GenericMessenger;
class G4LogicalVolume;
namespace nexus {
  class SiPMpetFBK;
  class CylinderPointSampler;
}

namespace nexus {
  class FullRingInfinity : public BaseGeometry {

  public:
    // Constructor
    FullRingInfinity();
    //Destructor
    ~FullRingInfinity();

    /// Generate a vertex within a given region of the geometry
    G4ThreeVector GenerateVertex(const G4String& region) const;

    private:
    void Construct();
    void BuildCryostat();
    void BuildQuadSensors();
    void BuildSensors(); 
    void BuildPhantom(); 

    SiPMpetFBK* sipm_;
    
    G4LogicalVolume* lab_logic_;
    G4LogicalVolume* LXe_logic_;
    G4LogicalVolume* active_logic_;

    /// Messenger for the definition of control commands
    G4GenericMessenger* msg_; 

    G4double lat_dimension_cell_;
    G4double sipm_pitch_;
    G4int n_cells_; ///< number of virtual cells of ~ 5 cm of side I want to fit in the ring
    G4int lin_n_sipm_per_cell_; ///< linear number of sipms in a cell (the side, not the area)
    G4int instr_faces_; ///< number of instrumented faces
    G4double kapton_thickn_;
    G4double depth_;

    G4double internal_radius_, external_radius_;
    G4double cryo_width_, cryo_thickn_;

    G4double phantom_diam_; 
    G4double phantom_length_;

    G4double max_step_size_;

    CylinderPointSampler* cylindric_gen_;

  };
}
#endif