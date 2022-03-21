// ----------------------------------------------------------------------------
// nexus | test_geometry.h
//
// Simple cylindrical geometry for nexus exercise.
//
// The NEXT Collaboration
// ----------------------------------------------------------------------------

#ifndef TEST_GEOMETRY_H
#define TEST_GEOMETRY_H

#include "GeometryBase.h"
#include <vector>

class G4Material;
class G4GenericMessenger;
namespace nexus { class SpherePointSampler; }


namespace nexus {

  /// Cylindrical chamber filled with gas xenon

  class test_geometry: public GeometryBase
  {
  public:
    /// Constructor
    test_geometry();
    /// Destructor
    ~test_geometry();

    /// Return vertex within region <region> of the chamber
    G4ThreeVector GenerateVertex(const G4String& region) const;

    void Construct();

  private:
    G4double pressure_; ///< Pressure of the gas
    G4double radius_;   ///< Radius of the cylinder
    G4double length_;   ///< Length of the cylinder
    G4double thickn_;   ///< Thickness of the cylinder

    ///Function and variables to generate PMT positions
    void GeneratePositions();
    const G4int num_PMTs_;
    std::vector<G4ThreeVector> pmt_positions_;
    
    /// Vertexes random generator
    SpherePointSampler* sphere_vertex_gen_;

    /// Messenger for the definition of control commands
    G4GenericMessenger* msg_;
  };

} // end namespace nexus

#endif
