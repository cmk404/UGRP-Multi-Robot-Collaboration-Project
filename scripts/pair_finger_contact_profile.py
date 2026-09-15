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
