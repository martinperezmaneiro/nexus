// ----------------------------------------------------------------------------
// nexus | test_generator.cc
//
// 83mKr primary generator for
// the nexus exercise (100% EC).
//
// The NEXT Collaboration
// ----------------------------------------------------------------------------

#include "test_generator.h"

#include "DetectorConstruction.h"
#include "GeometryBase.h"
#include "FactoryBase.h"

#include <G4Event.hh>
#include <G4GenericMessenger.hh>
#include <G4RunManager.hh>
#include <G4ParticleTable.hh>
#include <G4RandomDirection.hh>
#include <Randomize.hh>

#include "CLHEP/Units/SystemOfUnits.h"
#include "CLHEP/Units/PhysicalConstants.h"

using namespace nexus;

REGISTER_CLASS(test_generator, G4VPrimaryGenerator)

namespace nexus {

  using namespace CLHEP;

  test_generator::test_generator():
  geom_(0), energy_32_(32.1473*keV),
  energy_9_(9.396*keV), lifetime_9_(154.*ns)
  {

     msg_ = new G4GenericMessenger(this, "/Generator/test_generator/",
    "Control commands of this Kr83 generator.");

     msg_->DeclareProperty("region", region_,
			   "Set the region of the geometry where the vertex will be generated.");

     // Set particle type searching in particle table by name
    particle_defelectron_ = G4ParticleTable::GetParticleTable()->
      FindParticle("e-");

    DetectorConstruction* detconst = (DetectorConstruction*)
      G4RunManager::GetRunManager()->GetUserDetectorConstruction();
    geom_ = detconst->GetGeometry();
  }

  test_generator::~test_generator()
  {
  }

  void test_generator::GeneratePrimaryVertex(G4Event* evt)
  {

    // Ask the geometry to generate a position for the particle
    G4ThreeVector position = geom_->GenerateVertex(region_);
   //
   // First transition (32 kEv) Always one electron. Set it's kinetic energy.
   //

    double eKin32 = energy_32_;

    G4double mass = particle_defelectron_->GetPDGMass();
   // Calculate cartesian components of momentum for the most energetic EC
    G4double energy = eKin32 + mass;
    G4double pmod = std::sqrt(energy*energy - mass*mass);
    G4ThreeVector momentum_dir32 = G4RandomDirection();
    G4double px = pmod * momentum_dir32.x();
    G4double py = pmod * momentum_dir32.y();
    G4double pz = pmod * momentum_dir32.z();
    // Set starting time of generation
    G4double time = 0;

    G4PrimaryVertex* vertex =
      new G4PrimaryVertex(position, time);

    // create new primaries and set them to the vertex

    G4PrimaryParticle* particle1 =
      new G4PrimaryParticle(particle_defelectron_);
    particle1->SetMomentum(px, py, pz);
    particle1->SetPolarization(0.,0.,0.);
    particle1->SetProperTime(time);
    vertex->SetPrimary(particle1);

    //
    // finite lifetime for the 9 keV line
    //
    const double time9 = lifetime_9_ * G4RandExponential::shoot();
    // Create the secondary EC
    // Calculate cartesian components of momentum for the most energetic EC
    energy = energy_9_ + mass;
    pmod = std::sqrt(energy*energy - mass*mass);
    G4ThreeVector momentum_dir9 = G4RandomDirection();
    px = pmod * momentum_dir9.x();
    py = pmod * momentum_dir9.y();
    pz = pmod * momentum_dir9.z();

    // Create primaries and set to the vertex

    G4PrimaryParticle* particle2 =
      new G4PrimaryParticle(particle_defelectron_);
    particle2->SetMomentum(px, py, pz);
    particle2->SetProperTime(time9);
    vertex->SetPrimary(particle2);

    evt->AddPrimaryVertex(vertex);
   }
  }
 // Name space nexus
