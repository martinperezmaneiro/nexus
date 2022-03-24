// ----------------------------------------------------------------------------
// nexus | test_generator.h
//
// 83mKr primary generator for
// the nexus exercise (100% EC).
//
// The NEXT Collaboration
// ----------------------------------------------------------------------------

#ifndef TEST_GENERATOR_H
#define TEST_GENERATOR_H

#include <vector>
#include <G4VPrimaryGenerator.hh>

class G4Event;
class G4ParticleDefinition;
class G4GenericMessenger;

namespace nexus {

  class GeometryBase;

  class test_generator: public G4VPrimaryGenerator
  {
  public:
    //Constructor
    test_generator();
    //Destructor
    ~test_generator();

    void GeneratePrimaryVertex(G4Event* evt);

  private:

    G4GenericMessenger* msg_;
    const GeometryBase* geom_;

    G4double energy_32_; // Transition energy from the 1/2- state to the intermediate state, 7/2+
    G4double energy_9_; // ... from the JP 7/2+ to the Kr83 fundamental state.
    G4double lifetime_9_; // ...The lifetime of the intermediate state.

    G4String region_;
    G4ParticleDefinition*  particle_defelectron_;
  };

}// end namespace nexus
#endif
