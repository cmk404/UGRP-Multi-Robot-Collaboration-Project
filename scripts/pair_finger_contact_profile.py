"""Static, explicit friction damping for the four finger/beam contacts only."""
import xml.etree.ElementTree as ET

def configure_finger_contacts(root, damping):
    if damping not in (0,3000):raise ValueError('explicit finger friction damping must be 0 or 3000')
    if not damping:return
    import mujoco
    import numpy as np
    # Resolve defaults from the authored model, not live simulation state.
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    contact=root.find('contact')
    if contact is None:contact=ET.SubElement(root,'contact')
    for rid in ('r1','r3'):
        for side in ('left','right'):
            names=(rid+'__'+side+'_finger','team_beam_geom');a,b=[m.geom(n).id for n in names]
            if m.geom_priority[a]!=m.geom_priority[b] or np.any(m.geom_solref[[a,b]]<=0):
                raise ValueError('unsupported normal contact mixing; review profile')
            weight=m.geom_solmix[a]/(m.geom_solmix[a]+m.geom_solmix[b])
            friction=np.maximum(m.geom_friction[a],m.geom_friction[b])
            values={'friction':friction[[0,0,1,2,2]],'solref':weight*m.geom_solref[a]+(1-weight)*m.geom_solref[b],
                    'solimp':weight*m.geom_solimp[a]+(1-weight)*m.geom_solimp[b]}
            ET.SubElement(contact,'pair',name=rid+'_'+side+'_beam_contact',geom1=names[0],geom2=names[1],
                condim=str(max(m.geom_condim[a],m.geom_condim[b])),margin=str(max(m.geom_margin[a],m.geom_margin[b])),
                gap=str(max(m.geom_gap[a],m.geom_gap[b])),solreffriction=f'0 {-damping}',
                **{k:' '.join(str(float(x)) for x in v) for k,v in values.items()})

def contact_profile_record(m):
    fields=('pair_dim','pair_geom1','pair_geom2','pair_solref','pair_solreffriction','pair_solimp','pair_margin','pair_gap','pair_friction')
    return {name:getattr(m,name).tolist() for name in fields}

def audit_contact_profile(scene_path, report):
    """Compile saved static XML for output auditing, never for an actor."""
    if 'contact_profile' not in report:return  # Legacy records retain their own audit scope.
    import mujoco
    import numpy as np
    m=mujoco.MjModel.from_xml_path(str(scene_path))
    profile=report['contact_profile']
    if profile not in ('baseline','retention'):raise ValueError('unknown contact profile')
    if m.opt.timestep != (.00025 if profile=='retention' else .002) or m.opt.noslip_iterations!=0:
        raise ValueError('declared contact solver differs from saved XML')
    if contact_profile_record(m)!=report['invariants_initial']['explicit_contact_pairs']:
        raise ValueError('contact pair record differs from saved XML')
    expected={(rid+'__'+side+'_finger','team_beam_geom') for rid in ('r1','r3') for side in ('left','right')}
    actual={(m.geom(int(a)).name,m.geom(int(b)).name) for a,b in zip(m.pair_geom1,m.pair_geom2)}
    if profile=='baseline':
        if actual:raise ValueError('baseline contains explicit contact overrides')
        return
    if actual!=expected or m.npair!=4 or m.opt.impratio!=10 or not np.all(m.pair_solreffriction==[0,-3000]):
        raise ValueError('retention contact scope mismatch')
    # Verify the explicit profile preserved each contact's authored normal law
    # and friction capacity. Only its tangential regularization was changed.
    for i,(a,b) in enumerate(zip(m.pair_geom1,m.pair_geom2)):
        weight=m.geom_solmix[a]/(m.geom_solmix[a]+m.geom_solmix[b])
        for pair,geom in ((m.pair_solref,m.geom_solref),(m.pair_solimp,m.geom_solimp)):
            if not np.allclose(pair[i],weight*geom[a]+(1-weight)*geom[b],rtol=0,atol=1e-12):
                raise ValueError('normal contact law changed')
        if not np.array_equal(m.pair_friction[i],np.maximum(m.geom_friction[a],m.geom_friction[b])[[0,0,1,2,2]]):
            raise ValueError('friction capacity changed')
