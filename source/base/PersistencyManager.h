// ----------------------------------------------------------------------------
///  \file   PersistencyManager.h
///  \brief  
/// 
///  \author   <justo.martin-albo@ific.uv.es>
///  \date     13 March 2013
///  \version  $Id$
///
///  Copyright (c) 2013 NEXT Collaboration. All rights reserved.
// ----------------------------------------------------------------------------

#ifndef PERSISTENCY_MANAGER_H
#define PERSISTENCY_MANAGER_H

#include <G4VPersistencyManager.hh>
#include <map>


class G4GenericMessenger;
class G4TrajectoryContainer;
class G4HCofThisEvent;
class G4VHitsCollection;
namespace gate { class Event; }
namespace gate { class MCParticle; }
namespace gate { class MCTrack; }
namespace gate { class RootWriter; }


namespace nexus {


  /// TODO. CLASS DESCRIPTION

  class PersistencyManager: public G4VPersistencyManager
  {
  public:
    /// Create the singleton instance of the persistency manager
    static void Initialize();

    /// Set whether to store or not the current event
    void StoreCurrentEvent(G4bool);

    /// Set to the PM the name of the logical volume of the main geometry
    /// for hit coordinate conversion if needed
    void SetDriftCoord();

    /// 
    virtual G4bool Store(const G4Event*);
    virtual G4bool Store(const G4Run*);
    virtual G4bool Store(const G4VPhysicalVolume*);

    virtual G4bool Retrieve(G4Event*&);
    virtual G4bool Retrieve(G4Run*&);
    virtual G4bool Retrieve(G4VPhysicalVolume*&);

  public:
    void OpenFile(G4String);
    void CloseFile();


  private:
    PersistencyManager();
    ~PersistencyManager();
    PersistencyManager(const PersistencyManager&);

    void StoreTrajectories(G4TrajectoryContainer*, gate::Event*);
    void StoreHits(G4HCofThisEvent*, gate::Event*);
    void StoreIonizationHits(G4VHitsCollection*, gate::Event*);
    void StorePmtHits(G4VHitsCollection*, gate::Event*);


  private:
    G4GenericMessenger* _msg; ///< User configuration messenger

    G4String _historyFile;

    G4bool _ready;     ///< Is the PersistencyManager ready to go?
    G4bool _store_evt; ///< Should we store the current event?

    G4String event_type_; ///< event type: bb0nu, bb2nu, background or not set

    // gate::Event* _evt;         ///< Persistent gate event
    gate::RootWriter* _writer; ///< Event writer to ROOT file

    std::map<G4int, gate::MCParticle*> _iprtmap;
    std::map<G4int, gate::MCTrack*> _itrkmap;

    G4double _el_starting_z; ///< z where EL photons start to be generated
    G4bool _drift_z; ///< true if we want to write the z drift coordinate instead of true nexus

    G4int _saved_evts;
  };


  // INLINE DEFINITIONS //////////////////////////////////////////////

  inline void PersistencyManager::StoreCurrentEvent(G4bool sce)
  { _store_evt = sce; }
  inline G4bool PersistencyManager::Store(const G4VPhysicalVolume*)
  { return false; }
  inline G4bool PersistencyManager::Retrieve(G4Event*&) 
  { return false; }
  inline G4bool PersistencyManager::Retrieve(G4Run*&) 
  { return false; }
  inline G4bool PersistencyManager::Retrieve(G4VPhysicalVolume*&) 
  { return false; }

} // namespace nexus

#endif
