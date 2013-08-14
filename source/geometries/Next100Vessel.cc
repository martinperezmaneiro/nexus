// ----------------------------------------------------------------------------
//  $Id$
//
//  Authors: <justo.martin-albo@ific.uv.es>, <jmunoz@ific.uv.es>
//  Created: 21 Nov 2011
//  
//  Copyright (c) 2011 NEXT Collaboration
// ---------------------------------------------------------------------------- 

#include "Next100Vessel.h"
#include "MaterialsList.h"
#include "ConfigService.h"
#include "OpticalMaterialProperties.h"

#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4VisAttributes.hh>
#include <G4UnionSolid.hh>
#include <G4Tubs.hh>
#include <G4Sphere.hh>
#include <G4NistManager.hh>
#include <G4Material.hh>
#include <G4Colour.hh>
#include <Randomize.hh>
#include <G4TransportationManager.hh>


namespace nexus {

  
  Next100Vessel::Next100Vessel(const G4double nozzle_ext_diam,
			       const G4double up_nozzle_ypos,
			       const G4double central_nozzle_ypos,
			       const G4double down_nozzle_ypos,
			       const G4double bottom_nozzle_ypos):
    BaseGeometry(),

    // Body dimensions
    _vessel_in_rad (68.0  * cm),
    _vessel_body_length (160 * cm),
    _vessel_length (181.74 * cm),  // Vessel length = 160 cm (from body) + 2. * 10.87cm (from endcaps)
    _vessel_thickness (1.  * cm),

    // Endcaps dimensions
    _endcap_in_rad (108.94 * cm),
    _endcap_theta (38.6 * deg),
    _endcap_thickness (1. * cm),
    _endcap_in_z_width (23.83 * cm),  // in_z_width = 35.7 cm - 1. cm (thickness) - 10.87 cm (from cylindric part)

    // Flange dimensions
    _flange_out_rad (73.5 * cm),
    _flange_length (8.0 * cm),
    _flange_z_pos (80.0 * cm),

    // Nozzle dimensions
    _large_nozzle_length (320.0 * cm),
    _small_nozzle_length (240.0 * cm),

    // Vessel gas
    _temperature (303 * kelvin)
  {

    ReadParameters();

    // Initializing the geometry navigator (used in vertex generation)
    _geom_navigator = G4TransportationManager::GetTransportationManager()->GetNavigatorForTracking();


    // VESSEL   ///////////

    // Body solid
    G4double vessel_out_rad = _vessel_in_rad + _vessel_thickness;

    G4Tubs* vessel_body_solid = new G4Tubs("VESSEL_BODY", 0., vessel_out_rad, _vessel_length/2.,
					   0.*deg, 360.*deg);

    G4Tubs* vessel_gas_body_solid = new G4Tubs("VESSEL_GAS_BODY", 0., _vessel_in_rad, _vessel_length/2.,
					       0.*deg, 360.*deg);

    // Endcaps solids
    G4double endcap_out_rad = _endcap_in_rad + _endcap_thickness;

    G4Sphere* vessel_tracking_endcap_solid = new G4Sphere("VESSEL_TRACKING_ENDCAP",
							  0. * cm,  endcap_out_rad,   //radius
							  0. * deg, 360. * deg,       // phi
							  0. * deg, _endcap_theta);   // theta

    G4Sphere* vessel_gas_tracking_endcap_solid = new G4Sphere("VESSEL_GAS_TRACKING_ENDCAP",
							      0. * cm,  _endcap_in_rad,   //radius
							      0. * deg, 360. * deg,       // phi
							      0. * deg, _endcap_theta);   // theta

    G4Sphere* vessel_energy_endcap_solid = new G4Sphere("VESSEL_ENERGY_ENDCAP",
							0. * cm,  endcap_out_rad,                   //radius
							0. * deg, 360. * deg,                       // phi
							180. * deg - _endcap_theta, _endcap_theta); // theta

    G4Sphere* vessel_gas_energy_endcap_solid = new G4Sphere("VESSEL_GAS_ENERGY_ENDCAP",
							    0. * cm,  _endcap_in_rad,                   //radius
							    0. * deg, 360. * deg,                       // phi
							    180. * deg - _endcap_theta, _endcap_theta); // theta


    // Flange solid
    G4Tubs* vessel_flange_solid = new G4Tubs("VESSEL_TRACKING_FLANGE", 0., _flange_out_rad,
						      _flange_length/2., 0.*deg, 360.*deg);

    // Nozzle solids
    G4Tubs* large_nozzle_solid = new G4Tubs("LARGE_NOZZLE", 0.*cm, nozzle_ext_diam/2.,
					    _large_nozzle_length/2., 0.*deg, 360.*deg);

    G4Tubs* small_nozzle_solid = new G4Tubs("SMALL_NOZZLE", 0.*cm, nozzle_ext_diam/2.,
					    _small_nozzle_length/2., 0.*deg, 360.*deg);

    G4Tubs* large_nozzle_gas_solid = new G4Tubs("LARGE_NOZZLE_GAS", 0.*cm, (nozzle_ext_diam/2. - _vessel_thickness),
						_large_nozzle_length/2., 0.*deg, 360.*deg);

    G4Tubs* small_nozzle_gas_solid = new G4Tubs("SMALL_NOZZLE_GAS", 0.*cm, (nozzle_ext_diam/2. - _vessel_thickness),
						_small_nozzle_length/2., 0.*deg, 360.*deg);


    //// Unions
    G4double endcap_z_pos = (_vessel_length / 2.) - (_endcap_in_rad - _endcap_in_z_width);
    G4ThreeVector tracking_endcap_pos(0, 0, endcap_z_pos);
    G4ThreeVector energy_endcap_pos(0, 0, -1. * endcap_z_pos);
    G4ThreeVector tracking_flange_pos(0, 0, _flange_z_pos);
    G4ThreeVector energy_flange_pos(0, 0, -1. * _flange_z_pos);

    // Body + Tracking endcap
    G4UnionSolid* vessel_solid = new G4UnionSolid("VESSEL", vessel_body_solid, vessel_tracking_endcap_solid,
						  0, tracking_endcap_pos);

    // Body + Tracking endcap + Energy endcap
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid, vessel_energy_endcap_solid,
				    0, energy_endcap_pos);

    // Body + Tracking endcap + Energy endcap + Tracking flange
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid,
				    vessel_flange_solid, 0, tracking_flange_pos);

    // Body + Tracking endcap + Energy endcap + Tracking flange + Energy flange
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid,
				    vessel_flange_solid, 0, energy_flange_pos);

    // Adding nozzles
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid, small_nozzle_solid,
				    0, G4ThreeVector(0., up_nozzle_ypos, 0.) );
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid, large_nozzle_solid,
				    0, G4ThreeVector(0., central_nozzle_ypos, 0.) );
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid, large_nozzle_solid,
				    0, G4ThreeVector(0., down_nozzle_ypos, 0.) );
    vessel_solid = new G4UnionSolid("VESSEL", vessel_solid, small_nozzle_solid,
				    0, G4ThreeVector(0., bottom_nozzle_ypos, 0.) );



    // Body gas + Tracking endcap gas
    G4UnionSolid* vessel_gas_solid = new G4UnionSolid("VESSEL_GAS", vessel_gas_body_solid,
						      vessel_gas_tracking_endcap_solid, 0, tracking_endcap_pos);
    
    // Body gas + Tracking endcap gas + Energy endcap gas
    vessel_gas_solid = new G4UnionSolid("VESSEL_GAS", vessel_gas_solid,
					vessel_gas_energy_endcap_solid, 0, energy_endcap_pos);
    // Adding nozzles
    vessel_gas_solid = new G4UnionSolid("VESSEL_GAS", vessel_gas_solid, small_nozzle_gas_solid,
					0, G4ThreeVector(0., up_nozzle_ypos, 0.) );
    vessel_gas_solid = new G4UnionSolid("VESSEL_GAS", vessel_gas_solid, large_nozzle_gas_solid,
					0, G4ThreeVector(0., central_nozzle_ypos, 0.) );
    vessel_gas_solid = new G4UnionSolid("VESSEL_GAS", vessel_gas_solid, large_nozzle_gas_solid,
					0, G4ThreeVector(0., down_nozzle_ypos, 0.) );
    vessel_gas_solid = new G4UnionSolid("VESSEL_GAS", vessel_gas_solid, small_nozzle_gas_solid,
					0, G4ThreeVector(0., bottom_nozzle_ypos, 0.) );



    //// The logics
    G4LogicalVolume* vessel_logic = new G4LogicalVolume(vessel_solid,
							MaterialsList::Steel316Ti(),
							"VESSEL");
    this->SetLogicalVolume(vessel_logic);

    G4Material* vessel_gas_mat =  MaterialsList::GXe(_pressure, _temperature);
    vessel_gas_mat->SetMaterialPropertiesTable(OpticalMaterialProperties::GXe(_pressure, _temperature));

    G4LogicalVolume* vessel_gas_logic = new G4LogicalVolume(vessel_gas_solid, vessel_gas_mat, "VESSEL_GAS");
    _internal_logic_vol = vessel_gas_logic;

    G4PVPlacement* vessel_gas_physi = new G4PVPlacement(0, G4ThreeVector(0.,0.,0.), vessel_gas_logic,
							"VESSEL_GAS", vessel_logic, false, 0);


    //// Vacuum Manifold
    G4double vacuum_manifold_rad = nozzle_ext_diam/2. - _vessel_thickness;
    G4double vacuum_manifold_length = (_large_nozzle_length - _vessel_body_length) / 2. + 9.4*cm; // 10.6 cm comes from Derek's drawings
                                                                                                  // Switched to 9.4 to be aligned with Energy Plane
    G4double vacuum_manifold_zpos = -1. * (_large_nozzle_length - vacuum_manifold_length) / 2.;

    G4Tubs* vacuum_manifold_solid = new G4Tubs("VACUUM_MANIFOLD", 0.*cm, vacuum_manifold_rad,
					       vacuum_manifold_length/2., 0.*deg, 360.*deg);

    G4LogicalVolume* vacuum_manifold_logic = new G4LogicalVolume(vacuum_manifold_solid,
								 G4NistManager::Instance()->FindOrBuildMaterial("G4_Cu"),
								 "VACUUM_MANIFOLD");

    G4PVPlacement* vacuum_manifold_physi = new G4PVPlacement(0, G4ThreeVector(0.,0.,vacuum_manifold_zpos), vacuum_manifold_logic,
							     "VACUUM_MANIFOLD", vessel_gas_logic, false, 0);


    G4double vacuum_manifold_gas_rad = vacuum_manifold_rad - _vessel_thickness;
    G4double vacuum_manifold_gas_length = vacuum_manifold_length - 2. * _vessel_thickness;

    G4Tubs* vacuum_manifold_gas_solid = new G4Tubs("VACUUM_MANIFOLD_GAS", 0.*cm, vacuum_manifold_gas_rad,
						   vacuum_manifold_gas_length/2., 0.*deg, 360.*deg);

    G4LogicalVolume* vacuum_manifold_gas_logic = new G4LogicalVolume(vacuum_manifold_gas_solid,
								     G4NistManager::Instance()->FindOrBuildMaterial("G4_Galactic"),
								     "VACUUM_MANIFOLD_GAS");

    G4PVPlacement* vacuum_manifold_gas_physi = new G4PVPlacement(0, G4ThreeVector(0.,0.,0.), vacuum_manifold_gas_logic,
								 "VACUUM_MANIFOLD_GAS", vacuum_manifold_logic, false, 0);



    // SETTING VISIBILITIES   //////////
    if (_visibility) {
      G4VisAttributes grey(G4Colour(.7, .7, .7));
      //grey.SetForceSolid(true);
      vessel_logic->SetVisAttributes(grey);
      //grey.SetForceSolid(true);
      vessel_gas_logic->SetVisAttributes(grey);
      //grey.SetForceSolid(true);
      vacuum_manifold_logic->SetVisAttributes(grey);
      //grey.SetForceSolid(true);
      vacuum_manifold_gas_logic->SetVisAttributes(grey);
    }
    else {
      vessel_logic->SetVisAttributes(G4VisAttributes::Invisible);
      vessel_gas_logic->SetVisAttributes(G4VisAttributes::Invisible);
      vacuum_manifold_logic->SetVisAttributes(G4VisAttributes::Invisible);
      vacuum_manifold_gas_logic->SetVisAttributes(G4VisAttributes::Invisible);
    }


    // VERTEX GENERATORS   //////////
    _body_gen  = new CylinderPointSampler(_vessel_in_rad, _vessel_length, _vessel_thickness, 0.);

    _tracking_endcap_gen = new SpherePointSampler( _endcap_in_rad, _endcap_thickness, tracking_endcap_pos, 0,
						   0., twopi, 0., _endcap_theta);

    _energy_endcap_gen = new SpherePointSampler( _endcap_in_rad, _endcap_thickness, energy_endcap_pos, 0,
						 0., twopi, 180.*deg - _endcap_theta, _endcap_theta);

    _tracking_flange_gen  = new CylinderPointSampler(vessel_out_rad, _flange_length,
						     _flange_out_rad-vessel_out_rad, 0., tracking_flange_pos);

    _energy_flange_gen  = new CylinderPointSampler(vessel_out_rad, _flange_length,
						   _flange_out_rad-vessel_out_rad, 0., energy_flange_pos);

    // Calculating some prob
    G4double body_vol = vessel_body_solid->GetCubicVolume() - vessel_gas_body_solid->GetCubicVolume();
    G4double endcap_vol =  vessel_tracking_endcap_solid->GetCubicVolume() - vessel_gas_tracking_endcap_solid->GetCubicVolume();
    _perc_endcap_vol = endcap_vol / (body_vol + 2. * endcap_vol);
  }
  

  
  Next100Vessel::~Next100Vessel()
  {
    delete _body_gen;
    delete _tracking_endcap_gen;
    delete _energy_endcap_gen;
    delete _tracking_flange_gen;
    delete _energy_flange_gen;
  }
  


  void Next100Vessel::ReadParameters()
  {
    const ParamStore& cfg_geom = ConfigService::Instance().Geometry();
    _visibility = cfg_geom.GetIParam("vessel_vis");
    _pressure = cfg_geom.GetDParam("pressure");
    
  }



  G4LogicalVolume* Next100Vessel::GetInternalLogicalVolume()
  {
    return _internal_logic_vol;
  }



  G4ThreeVector Next100Vessel::GenerateVertex(const G4String& region) const
  {
    G4ThreeVector vertex(0., 0., 0.);

    // Vertex in the whole VESSEL volume except flanges
    if (region == "VESSEL") {
      G4double rand = G4UniformRand();
      if (rand < _perc_endcap_vol) {
	G4VPhysicalVolume *VertexVolume;
	do {
	  vertex = _tracking_endcap_gen->GenerateVertex("VOLUME");  // Tracking endcap
	  VertexVolume = _geom_navigator->LocateGlobalPointAndSetup(vertex, 0, false);
	} while (VertexVolume->GetName() != "VESSEL");
      }
      else if (rand > 1. - _perc_endcap_vol) {
	G4VPhysicalVolume *VertexVolume;
	do {
	  vertex = _energy_endcap_gen->GenerateVertex("VOLUME");  // Energy endcap
	  VertexVolume = _geom_navigator->LocateGlobalPointAndSetup(vertex, 0, false);
	} while (VertexVolume->GetName() != "VESSEL");
      }
      else
	vertex = _body_gen->GenerateVertex(TUBE_VOLUME);  // Body
    }

    // Vertex in FLANGES
    else if (region == "VESSEL_FLANGES") {
      if (G4UniformRand() < 0.5)
      	vertex = _tracking_flange_gen->GenerateVertex(TUBE_VOLUME);
      else
      	vertex = _energy_flange_gen->GenerateVertex(TUBE_VOLUME);
    }

    // Vertex in TRACKING ENDCAP
    else if (region == "VESSEL_TRACKING_ENDCAP") {
      G4VPhysicalVolume *VertexVolume;
      do {
	vertex = _tracking_endcap_gen->GenerateVertex("VOLUME");  // Tracking endcap
	VertexVolume = _geom_navigator->LocateGlobalPointAndSetup(vertex, 0, false);
      } while (VertexVolume->GetName() != "VESSEL");
    }

    // Vertex in ENERGY ENDCAP
    else if (region == "VESSEL_ENERGY_ENDCAP") {
      G4VPhysicalVolume *VertexVolume;
      do {
	vertex = _energy_endcap_gen->GenerateVertex("VOLUME");  // Energy endcap
	VertexVolume = _geom_navigator->LocateGlobalPointAndSetup(vertex, 0, false);
      } while (VertexVolume->GetName() != "VESSEL");
    }

    return vertex;
  }


} //end namespace nexus
