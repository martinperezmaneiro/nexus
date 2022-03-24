// ----------------------------------------------------------------------------
// nexus | test_geometry.cc
//
// Simple cylindrical geometry for nexus exercise.
//
// The NEXT Collaboration
// ----------------------------------------------------------------------------

#include "test_geometry.h"

#include "PmtR11410.h"
#include "CylinderPointSampler2020.h"
#include "MaterialsList.h"
#include "IonizationSD.h"
#include "FactoryBase.h"

#include <G4GenericMessenger.hh>
#include <G4Navigator.hh>
#include <G4Tubs.hh>
#include <G4NistManager.hh>
#include <G4LogicalVolume.hh>
#include <G4VPhysicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4ThreeVector.hh>
#include <G4Material.hh>
#include <G4VisAttributes.hh>
#include <G4SDManager.hh>
#include <G4VUserDetectorConstruction.hh>
#include <G4TransportationManager.hh>

#include <CLHEP/Units/SystemOfUnits.h>

using namespace nexus;
using namespace CLHEP;

REGISTER_CLASS(test_geometry, GeometryBase)

namespace nexus {

  test_geometry::test_geometry():
    GeometryBase(), pressure_(STP_Pressure),
    radius_(100.*cm), length_(500.*cm), thickn_(1.*cm),
    cylinder_vertex_gen_(0), num_PMTs_(168)
  {

    /// Initializing the geometry navigator (used in vertex generation)
    geom_navigator_ =
      G4TransportationManager::GetTransportationManager()->GetNavigatorForTracking();

    /// Messenger
    msg_ = new G4GenericMessenger(this, "/Geometry/test_geometry/",
      "Control commands of test geometry.");

    G4GenericMessenger::Command& pressure_cmd =
      msg_->DeclareProperty("pressure", pressure_,
      "Set pressure for gaseous xenon (if selected).");
    pressure_cmd.SetUnitCategory("Pressure");
    pressure_cmd.SetParameterName("pressure", false);
    pressure_cmd.SetRange("pressure>0.");

    G4GenericMessenger::Command& radius_cmd =
      msg_->DeclareProperty("radius", radius_,
      "Radius of the xenon cylinder");
    radius_cmd.SetUnitCategory("Length");
    radius_cmd.SetParameterName("radius", false);
    radius_cmd.SetRange("radius>0.");

    G4GenericMessenger::Command& length_cmd =
      msg_->DeclareProperty("length", length_, "Lenght of the xenon cylinder.");
    length_cmd.SetUnitCategory("Length");
    length_cmd.SetParameterName("length", false);
    length_cmd.SetRange("length>0.");

    G4GenericMessenger::Command& thickn_cmd =
      msg_->DeclareProperty("thickness", thickn_, "Thickness of the chamber.");
    thickn_cmd.SetUnitCategory("Length");
    thickn_cmd.SetParameterName("thickness", false);
    thickn_cmd.SetRange("thickness>0.");
  }



  test_geometry::~test_geometry()
  {
    delete cylinder_vertex_gen_;
    delete msg_;
  }



  void test_geometry::Construct()
  {

    GeneratePositions();


    // CHAMBER
    G4Tubs* chamber_solid = new G4Tubs("CHAMBER", 0.*cm, (radius_ + thickn_),
                                          (length_/2. + thickn_), 0.*rad, 2. * M_PI * rad);

    G4LogicalVolume* chamber_logic = new G4LogicalVolume(chamber_solid,
                                                         materials::Steel(), "CHAMBER");

    GeometryBase::SetLogicalVolume(chamber_logic);

    // GAS
    G4Tubs* gas_solid = new G4Tubs("GAS", 0.*cm, radius_, length_/2., 0.*rad, 2. * M_PI * rad);

    G4Material* xenon = materials::GXe(pressure_);

    G4LogicalVolume* gas_logic = new G4LogicalVolume(gas_solid, xenon, "GAS");

    new G4PVPlacement(0, G4ThreeVector(0., 0., 0.), gas_logic, "GAS", chamber_logic,
                      false, 0, true);

    // PMTs
    PmtR11410* pmt_ = new PmtR11410();
    pmt_->SetSensorDepth(0);
    pmt_->Construct();
    G4LogicalVolume* pmt_logic = pmt_->GetLogicalVolume();

    G4double pmt_zpos = length_/2. - 38./2. * mm - 76. * mm;
    //being 38mm the length of the PMT front body and 76mm the length of the rear body

    //Allocation of al the pmt logical volumes
    G4ThreeVector pmt_pos = pmt_positions_[0];
    pmt_pos.setZ(-pmt_zpos);
    new G4PVPlacement(0, pmt_pos, pmt_logic, "PMT", gas_logic, false, 0, true);

    for (G4int i=1; i<num_PMTs_; i++) {
      pmt_pos = pmt_positions_[i];
      pmt_pos.setZ(-pmt_zpos);
      new G4PVPlacement(0, pmt_pos, pmt_logic, "PMT", gas_logic, false, 0, true);
    }

    // Create a vertex generator for a cylinder
    cylinder_vertex_gen_ = new CylinderPointSampler2020(0.*cm, radius_, length_/2., 0.*rad, 2. * M_PI * rad);

  }



  G4ThreeVector test_geometry::GenerateVertex(const G4String& region) const
  {
    G4ThreeVector vertex(0., 0., 0.);

      if (region == "GAS") {
        G4VPhysicalVolume *VertexVolume;
        do {
          vertex = cylinder_vertex_gen_->GenerateVertex("VOLUME");
          G4ThreeVector glob_vtx(vertex);
          glob_vtx = glob_vtx + G4ThreeVector(0., 0., 0.);
          VertexVolume =
            geom_navigator_->LocateGlobalPointAndSetup(glob_vtx, 0, false);
        } while (VertexVolume->GetName() != region);
      }

    return vertex;
  }

void test_geometry::GeneratePositions()
  {
    /// Function that computes and stores the XY positions of PMTs
    /// The number of needed PMTs is defined by P*C*(C+1)/2,
    /// being P the num_inner_pmts and C the num_conc_circles

    G4int num_conc_circles = 7;
    G4int num_inner_pmts = 6;
    G4double x_pitch = 125 * mm;
    G4double y_pitch = 108.3 * mm;
    G4int total_positions = 0;
    G4ThreeVector position(0.,0.,0.);

    for (G4int circle=1; circle<=num_conc_circles; circle++) {
      G4double rad     = circle * x_pitch;
      G4double step    = 360.0/num_inner_pmts;

      for (G4int place=0; place<num_inner_pmts; place++) {
      	G4double angle = place * step;
      	position.setX(rad * cos(angle*deg));
      	position.setY(rad * sin(angle*deg));
      	pmt_positions_.push_back(position);
      	total_positions++;
      }

      for (G4int i=1; i<circle; i++) {
      	G4double start_x = (circle-(i*0.5))*x_pitch;
      	G4double start_y = i*y_pitch;
      	rad  = std::sqrt(std::pow(start_x, 2) + std::pow(start_y, 2));
      	G4double start_angle = std::atan2(start_y, start_x)/deg;

      	for (G4int place=0; place<num_inner_pmts; place++) {
      	  G4double angle = start_angle + place * step;
      	  position.setX(rad * cos(angle*deg));
      	  position.setY(rad * sin(angle*deg));
      	  pmt_positions_.push_back(position);
      	  total_positions++;
	       }
      }
    }

    // Checking
    if (total_positions != num_PMTs_) {
      G4cout << "\nERROR: Number of PMTs doesn't match with number of positions calculated\n";
      G4cout << "Number of PMTs = " << num_PMTs_ << ", number of positions = "
	     << total_positions << G4endl;
      exit(0);
    }
  }

} // end namespace nexus
