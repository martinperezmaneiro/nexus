// ----------------------------------------------------------------------------
//  $Id$
//
//  Author:  <justo.martin-albo@ific.uv.es>
//  Created: 2 March 2010
//  
//  Copyright (c) 2010-2013 NEXT Collaboration. All rights reserved.
// ---------------------------------------------------------------------------- 

#include "SiPMpetVUV.h"
#include "PmtSD.h"
#include "MaterialsList.h"
#include <G4GenericMessenger.hh>
#include "OpticalMaterialProperties.h"

#include <G4Box.hh>
#include <G4LogicalVolume.hh>
#include <G4VisAttributes.hh>
#include <G4PVPlacement.hh>
#include <G4Material.hh>
#include <G4NistManager.hh>
#include <G4SDManager.hh>
#include <G4OpticalSurface.hh>
#include <G4LogicalSkinSurface.hh>
#include <G4PhysicalConstants.hh>

#include <CLHEP/Units/SystemOfUnits.h>


namespace nexus {

  using namespace CLHEP;
  
  SiPMpetVUV::SiPMpetVUV(): BaseGeometry(),
		    _visibility(0)

  {
    /// Messenger
    _msg = new G4GenericMessenger(this, "/Geometry/PetalX/", "Control commands of geometry.");
    _msg->DeclareProperty("SiPMpet_vis", _visibility, "SiPMpet Visibility");
  }
  
  
  
  SiPMpetVUV::~SiPMpetVUV()
  {
  }



  G4ThreeVector SiPMpetVUV::GetDimensions() const
  {
    return _dimensions;
  }
  
  
  
  void SiPMpetVUV::Construct()
  {
   
    // PACKAGE ///////////////////////////////////////////////////////
    
    // Hamamatsu 3X3MM-50UM VUV3
    // G4double sipm_x = 6.5* mm;
    // G4double sipm_y = 6.5 * mm;
    // G4double sipm_z = 1.55 * mm;

    // array-style
    G4double sipm_x = 3.* mm;
    G4double sipm_y = 3. * mm;
    G4double sipm_z = 1.55 * mm;

    _dimensions.setX(sipm_x);
    _dimensions.setY(sipm_y);
    _dimensions.setZ(sipm_z);

    G4Box* sipm_solid = new G4Box("SIPMpet", sipm_x/2., sipm_y/2., sipm_z/2);

    G4Material* epoxy = MaterialsList::Epoxy();
    epoxy->SetMaterialPropertiesTable(OpticalMaterialProperties::EpoxyVUV());
    
    G4LogicalVolume* sipm_logic = 
      new G4LogicalVolume(sipm_solid, epoxy, "SIPMpet");

    this->SetLogicalVolume(sipm_logic);


    // PCB ///////////////////////////////////////////////////////

    // G4double pcb_z = 0.550 * mm;
    
    // G4Material* plastic = G4NistManager::Instance()->FindOrBuildMaterial("G4_POLYCARBONATE");

    // G4Box* plastic_solid = new G4Box("PLASTIC", sipm_x/2., sipm_y/2., pcb_z/2);
    
    // G4LogicalVolume* plastic_logic = 
    // new G4LogicalVolume(plastic_solid, plastic, "PLASTIC");
    
    // G4double epoxy_z = 0.300 * mm;

    // new G4PVPlacement(0, G4ThreeVector(0, 0., epoxy_z/2), plastic_logic,
    // 		      "PLASTIC", sipm_logic, false, 0, false);

    // ACTIVE WINDOW /////////////////////////////////////////////////

    G4double active_side     = 3.0   * mm;
    G4double active_depth    = 0.01   * mm;
    
    G4Box* active_solid =
      new G4Box("PHOTODIODES", active_side/2., active_side/2., active_depth/2);
    
    G4Material* silicon = 
      G4NistManager::Instance()->FindOrBuildMaterial("G4_Si");

    G4LogicalVolume* active_logic =
      new G4LogicalVolume(active_solid, silicon, "PHOTODIODES");

    new G4PVPlacement(0, G4ThreeVector(0., 0., sipm_z/2. - active_depth/2. - .1*mm), active_logic,
		      "PHOTODIODES", sipm_logic, false, 0, false);
    
    
    // OPTICAL SURFACES //////////////////////////////////////////////

    // SiPM efficiency set using the official Hamamatsu specs.
    
    const G4int entries = 11;


     G4double energies[entries] = {6.19919*eV, 6.35814*eV, 6.52546*eV,
				   6.70182*eV, 6.88799*eV, 7.08479*eV,
				   7.29316*eV, 7.51417*eV, 7.74898*eV,
				   7.99895*eV, 8.26558*eV};
    G4double reflectivity[entries] = {0., 0., 0.,
				      0., 0., 0.,
                                      0., 0., 0.,      
				      0., 0.};
    // G4double efficiency[entries]   = {0.092, 0.09, 0.086, 
    //                                   0.082, 0.081, 0.08, 
    //                                   0.08,  0.09, 0.092,
    //                                   0.108, 0.130};
    G4double efficiency[entries]   = {1., 1., 1., 
                                      1., 1., 1., 
                                      1., 1., 1.,
                                      1., 1.};
    
    // G4double efficiency_red[entries];
    // for (G4int i=0; i<entries; ++i) {
    //   efficiency_red[i] = efficiency[i];
    // }


    
    G4MaterialPropertiesTable* sipm_mt = new G4MaterialPropertiesTable();
    //sipm_mt->AddProperty("EFFICIENCY", energies, efficiency_red, entries);
    sipm_mt->AddProperty("EFFICIENCY", energies, efficiency, entries);
    sipm_mt->AddProperty("REFLECTIVITY", energies, reflectivity, entries);

    G4OpticalSurface* sipm_opsurf = 
      new G4OpticalSurface("SIPM_OPSURF", unified, polished, dielectric_metal);
    sipm_opsurf->SetMaterialPropertiesTable(sipm_mt);

    new G4LogicalSkinSurface("SIPM_OPSURF", active_logic, sipm_opsurf);    
    
    
    // SENSITIVE DETECTOR ////////////////////////////////////////////

    G4String sdname = "/SIPM/SiPMpetVUV";
    G4SDManager* sdmgr = G4SDManager::GetSDMpointer();
    
    if (!sdmgr->FindSensitiveDetector(sdname, false)) {
      PmtSD* sipmsd = new PmtSD(sdname);
      sipmsd->SetDetectorVolumeDepth(0);
      sipmsd->SetDetectorNamingOrder(1000.);
      sipmsd->SetTimeBinning(5.*picosecond);
      sipmsd->SetMotherVolumeDepth(1);
      sipmsd->SetGrandMotherVolumeDepth(3);
      
      G4SDManager::GetSDMpointer()->AddNewDetector(sipmsd);
      sipm_logic->SetSensitiveDetector(sipmsd);
    }

    // Visibilities
    if (_visibility) {
      G4VisAttributes sipm_col(G4Colour(.40,.55,.55));
      sipm_logic->SetVisAttributes(sipm_col);
      G4VisAttributes active_col(G4Colour(1.,1.,0.));
      active_col.SetForceSolid(true);
      active_logic->SetVisAttributes(active_col);
    }
    else {
      sipm_logic->SetVisAttributes(G4VisAttributes::Invisible);
      active_logic->SetVisAttributes(G4VisAttributes::Invisible);
    }
  }
  
  
} // end namespace nexus
