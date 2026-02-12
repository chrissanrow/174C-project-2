import {tiny, defs} from './examples/common.js';

// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

const shapes = {
    'sphere': new defs.Subdivision_Sphere( 5 ),
};

class Node {
    constructor(name, shape, transform_matrix) {
        this.name = name;
        this.shape = shape;
        this.transform_matrix = transform_matrix;
        this.children_arcs = [];
    }
}

class Arc {
    constructor(name, parent_node, child_node, location_matrix) {
        this.name = name;
        this.parent_node = parent_node;
        this.child_node = child_node;
        this.location_matrix = location_matrix;
        this.articulation_matrix = Mat4.identity();
        this.end_effector = false;
        this.end_effector_offset = vec3(0, 0, 0, 1);
        this.dof = {
            Rx: false,
            Ry: false,
            Rz: false,
            // translation DOF only on root
            Tx: false,
            Ty: false,
            Tz: false,
        }
    }

    set_dof(Rx, Ry, Rz, Tx, Ty, Tz) {
        this.dof.Rx = Rx;
        this.dof.Ry = Ry;
        this.dof.Rz = Rz;
        this.dof.Tx = Tx;
        this.dof.Ty = Ty;
        this.dof.Tz = Tz;
    }

    // theta is 6x1 vector -> truncated to only contain relevant DOF
    update_articulation_matrix(theta) {
        this.articulation_matrix = Mat4.identity();
        let i = 0;
        if (this.dof.Rx) {
            this.articulation_matrix.post_multiply(Mat4.rotation(theta[i], 1, 0, 0));
            i++;
        }
        if (this.dof.Ry) {
            this.articulation_matrix.post_multiply(Mat4.rotation(theta[i], 0, 1, 0));
            i++;
        }
        if (this.dof.Rz) {
            this.articulation_matrix.post_multiply(Mat4.rotation(theta[i], 0, 0, 1));
            i++;
        }
        if (this.dof.Tx) {
            this.articulation_matrix.post_multiply(Mat4.translation(theta[i], 0, 0));
            i++;
        }
        if (this.dof.Ty) {
            this.articulation_matrix.post_multiply(Mat4.translation(0, theta[i], 0));
            i++;
        }
        if (this.dof.Tz) {
            this.articulation_matrix.post_multiply(Mat4.translation(0, 0, theta[i]));
            i++;
        }
    }
}

export const Human_Model =
class Human_Model {
    constructor() {
        const sphere_shape = shapes.sphere;

        // -- Link hierarchy setup --

        // torso node
        const torso_scale = Mat4.scale(1.25, 2, 0.5);
        this.torso_node = new Node("torso", sphere_shape, torso_scale);

        // root arc (no parent)
        const root_location = Mat4.translation(-1.5, 5.9, 0);
        this.root = new Arc("root", null, this.torso_node, root_location);

        // lower half of body
        
        // thigh nodes
        let l_thigh_scale = Mat4.scale(.4, 1, .4);
        l_thigh_scale.pre_multiply(Mat4.translation(0, -1, 0));
        this.l_thigh_node = new Node("l_thigh", sphere_shape, l_thigh_scale);

        let r_thigh_scale = Mat4.scale(.4, 1, .4);
        r_thigh_scale.pre_multiply(Mat4.translation(0, -1, 0));
        this.r_thigh_node = new Node("r_thigh", sphere_shape, r_thigh_scale);

        // hip arcs
        const lhip_location = Mat4.translation(-0.7, -1.7, 0);
        this.lhip = new Arc("lhip", this.torso_node, this.l_thigh_node, lhip_location);
        this.torso_node.children_arcs.push(this.lhip);

        const rhip_location = Mat4.translation(0.7, -1.7, 0);
        this.rhip = new Arc("rhip", this.torso_node, this.r_thigh_node, rhip_location);
        this.torso_node.children_arcs.push(this.rhip);

        // shin nodes
        let l_shin_scale = Mat4.scale(.4, 1, .4);
        l_shin_scale.pre_multiply(Mat4.translation(0, -1, 0));
        this.l_shin_node = new Node("l_shin", sphere_shape, l_shin_scale);

        let r_shin_scale = Mat4.scale(.4, 1, .4);
        r_shin_scale.pre_multiply(Mat4.translation(0, -1, 0));
        this.r_shin_node = new Node("r_shin", sphere_shape, r_shin_scale);
        
        // knee arcs
        const lknee_location = Mat4.translation(0, -2, 0);
        this.lknee = new Arc("lknee", this.l_thigh_node, this.l_shin_node, lknee_location);
        this.l_thigh_node.children_arcs.push(this.lknee);

        const rknee_location = Mat4.translation(0, -2, 0);
        this.rknee = new Arc("rknee", this.r_thigh_node, this.r_shin_node, rknee_location);
        this.r_thigh_node.children_arcs.push(this.rknee);

        // foot nodes
        let l_foot_scale = Mat4.scale(.4, .2, 0.5);
        l_foot_scale.pre_multiply(Mat4.translation(0, -0.1, -0.4));
        this.l_foot_node = new Node("l_foot", sphere_shape, l_foot_scale);

        let r_foot_scale = Mat4.scale(.4, .2, 0.5);
        r_foot_scale.pre_multiply(Mat4.translation(0, -0.1, -0.4));
        this.r_foot_node = new Node("r_foot", sphere_shape, r_foot_scale);

        // ankle arcs
        const lankle_location = Mat4.translation(0, -2, 0);
        this.lankle = new Arc("lankle", this.l_shin_node, this.l_foot_node, lankle_location);
        this.l_shin_node.children_arcs.push(this.lankle);

        const rankle_location = Mat4.translation(0, -2, 0);
        this.rankle = new Arc("rankle", this.r_shin_node, this.r_foot_node, rankle_location);
        this.r_shin_node.children_arcs.push(this.rankle);

        // upper half of body

        // head node
        let head_scale = Mat4.scale(.6, .75, .6);
        head_scale.pre_multiply(Mat4.translation(0, .75, 0));
        this.head_node = new Node("head", sphere_shape, head_scale);
        // neck arc
        const neck_location = Mat4.translation(0, 2, 0);
        this.neck = new Arc("neck", this.torso_node, this.head_node, neck_location);
        this.torso_node.children_arcs.push(this.neck);

        // upper arm nodes
        let ru_arm_scale = Mat4.scale(1.2, .3, .2);
        ru_arm_scale.pre_multiply(Mat4.translation(1.2, 0, 0));
        this.ru_arm_node = new Node("ru_arm", sphere_shape, ru_arm_scale);

        let lu_arm_scale = Mat4.scale(1.2, .3, .2);
        lu_arm_scale.pre_multiply(Mat4.translation(-1.2, 0, 0));
        this.lu_arm_node = new Node("lu_arm", sphere_shape, lu_arm_scale);

        // shoulder arcs
        const r_shoulder_location = Mat4.translation(0.75, 1.5, 0);
        this.r_shoulder = new Arc("r_shoulder", this.torso_node, this.ru_arm_node, r_shoulder_location);
        this.torso_node.children_arcs.push(this.r_shoulder);

        const l_shoulder_location = Mat4.translation(-0.75, 1.5, 0);
        this.l_shoulder = new Arc("l_shoulder", this.torso_node, this.lu_arm_node, l_shoulder_location);
        this.torso_node.children_arcs.push(this.l_shoulder);
        
        // lower arm nodes
        let rl_arm_scale = Mat4.scale(1, .3, .3);
        rl_arm_scale.pre_multiply(Mat4.translation(1, 0, 0));
        this.rl_arm_node = new Node("rl_arm", sphere_shape, rl_arm_scale);

        let ll_arm_scale = Mat4.scale(1, .3, .3);
        ll_arm_scale.pre_multiply(Mat4.translation(-1, 0, 0));
        this.ll_arm_node = new Node("ll_arm", sphere_shape, ll_arm_scale);
        
        // elbow arcs
        const r_elbow_location = Mat4.translation(2.4, 0, 0);
        this.r_elbow = new Arc("r_elbow", this.ru_arm_node, this.rl_arm_node, r_elbow_location);
        this.ru_arm_node.children_arcs.push(this.r_elbow);

        const l_elbow_location = Mat4.translation(-2.4, 0, 0);
        this.l_elbow = new Arc("l_elbow", this.lu_arm_node, this.ll_arm_node, l_elbow_location);
        this.lu_arm_node.children_arcs.push(this.l_elbow);

        // hand nodes
        let r_hand_scale = Mat4.scale(.4, .3, .3);
        r_hand_scale.pre_multiply(Mat4.translation(0.4, 0, 0));
        this.r_hand_node = new Node("r_hand", sphere_shape, r_hand_scale);
        // right hand is end effector
        this.r_hand_node.end_effector = true;
        this.r_hand_node.end_effector_offset = vec3(0.4, 0, 0, 1);

        let l_hand_scale = Mat4.scale(.4, .3, .2);
        l_hand_scale.pre_multiply(Mat4.translation(-0.4, 0, 0));
        this.l_hand_node = new Node("l_hand", sphere_shape, l_hand_scale);

        // wrist arcs
        const r_wrist_location = Mat4.translation(2, 0, 0);
        this.r_wrist = new Arc("r_wrist", this.rl_arm_node, this.r_hand_node, r_wrist_location);
        this.rl_arm_node.children_arcs.push(this.r_wrist);

        const l_wrist_location = Mat4.translation(-2, 0, 0);
        this.l_wrist = new Arc("l_wrist", this.ll_arm_node, this.l_hand_node, l_wrist_location);
        this.ll_arm_node.children_arcs.push(this.l_wrist);

        // -- DOF setup --
        // animation does not require DOF for lower half of body
        // TODO: include left arm in theta so we can create a natural pose
        // 3 translational at root
        this.root.set_dof(false, false, false, true, true, true);

        // 3 rotational at shoulders
        this.r_shoulder.set_dof(true, true, true, false, false, false);

        // 2 rotational at elbow (x and y)
        this.r_elbow.set_dof(true, true, false, false, false, false);

        // 2 rotational at wrist (y and z)
        this.r_wrist.set_dof(false, true, true, false, false, false);

        const kinematic_chain_size = 10; // for animation, only concerned with root -> rshoulder -> relbow -> rwrist

        this.theta = new Array(kinematic_chain_size).fill(0);
    }

    _forward_kinematics() {
        this.root.update_articulation_matrix(this.theta.slice(0, 3));
        this.r_shoulder.update_articulation_matrix(this.theta.slice(3, 6));
        this.r_elbow.update_articulation_matrix(this.theta.slice(6, 8));
        this.r_wrist.update_articulation_matrix(this.theta.slice(8, 10));
    }
    
    _get_end_effector_position(){
        // FK to get end effector position (same as _rec_update)
        if (arc !== null) {
            const L = arc.location_matrix;
            const A = arc.articulation_matrix;
            matrix.post_multiply(L.times(A));
            this.matrix_stack.push(matrix.copy());
            
            if(arc.child_node.end_effector) {
                const end_effector_pos = matrix.times(arc.child_node.end_effector_offset);
                return vec3(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2]);
            }
            
            const node = arc.child_node;
            const T = node.transform_matrix;
            matrix.post_multiply(T);
            node.shape.draw(webgl_manager, uniforms, matrix, material);
            
            matrix = this.matrix_stack.pop();
            for (const next_arc of node.children_arcs) {
                this.matrix_stack.push(matrix.copy());
                this._rec_draw(next_arc, matrix, webgl_manager, uniforms, material);
                matrix = this.matrix_stack.pop();
            }
        }
    }

    // from wikipedia (approximation of jacobian matrix): https://en.wikipedia.org/wiki/Inverse_kinematics#The_Jacobian_inverse_technique
    // partial(p_i)/partial(x_k) ~ (p_i(x_{0,k} + h) - p_i(x_{0,k}))/h, where x is equivalent to model's theta vector
    _compute_jacobian() {
        let J = [];
        for(let i = 0; i < 3; i++) {
            J.push([]);
        }

        const p_at_x0 = this._get_end_effector_position();
        const h = 0.001;

        for(let i = 0; i < 10; i++) {
            // compute p_i(x_{0,k} + h) using FK
            this.theta[i] += h;
            this._forward_kinematics();
            const p_at_x0_plus_h = this._get_end_effector_position();
            this.theta[i] -= h; // reset theta
            
            for(let k = 0; k < 3; k++) {
                J[k][i] = (p_at_x0_plus_h[k] - p_at_x0[k]) / h; // apply approximation
            }
        }
        return J;
    }

    draw(webgl_manager, uniforms, material) {
        this.matrix_stack = [];
        this._rec_draw(this.root, Mat4.identity(), webgl_manager, uniforms, material);
    }

    _rec_draw(arc, matrix, webgl_manager, uniforms, material) {
        if (arc !== null) {
            const L = arc.location_matrix;
            const A = arc.articulation_matrix;
            matrix.post_multiply(L.times(A));
            this.matrix_stack.push(matrix.copy());

            const node = arc.child_node;
            const T = node.transform_matrix;
            matrix.post_multiply(T);
            node.shape.draw(webgl_manager, uniforms, matrix, material);

            matrix = this.matrix_stack.pop();
            for (const next_arc of node.children_arcs) {
                this.matrix_stack.push(matrix.copy());
                this._rec_draw(next_arc, matrix, webgl_manager, uniforms, material);
                matrix = this.matrix_stack.pop();
            }
        }
    }

    debug(arc=null) {
        if (arc === null)
            arc = this.root;

        if (arc !== this.root) {
            arc.articulation_matrix = arc.articulation_matrix.times(Mat4.rotation(0.02, 0, 0, 1));
        }

        const node = arc.child_node;
        for (const next_arc of node.children_arcs) {
            this.debug(next_arc);
        }
    }
}