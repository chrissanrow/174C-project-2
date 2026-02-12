import {tiny, defs} from './examples/common.js';
// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

// Hermite spline class
export class Spline
{
  constructor()
  {
    this.points = [];
    this.tangents = [];
    this.num_control_points = 0;
  }

  add_control_point( x, y, z, mx, my, mz )
  {
    this.points.push(vec3(x, y, z));
    this.tangents.push(vec3(mx, my, mz));
    this.num_control_points++;
  }

  set_tangent( index, mx, my, mz )
  {
    this.tangents[index] = vec3(mx, my, mz);
  }

  set_point( index, x, y, z )
  {
    this.points[index] = vec3(x, y, z);
  }

  reset_spline(){
    this.points = [];
    this.tangents = [];
    this.num_control_points = 0;
  }
  
  piecewise_arc_length( num_samples = 1000 ){
    let lookup_table = new Arc_Length_Lookup();
    for(let i = 0; i <= num_samples; i++){
      const t = i / num_samples;
      const pos = this.get_position(t);
      lookup_table.i.push(i);
      lookup_table.u.push(t);
      if(i == 0){
        lookup_table.s.push(0);
      }
      else{
        const prev_pos = this.get_position((i - 1) / num_samples);
        const length = pos.minus(prev_pos).norm();
        lookup_table.s.push(lookup_table.s[i - 1] + length);
      }
    }
    return lookup_table;
  }

  h00(t){
    return 2*t*t*t - 3*t*t + 1;
  }

  h10(t){
    return t*t*t - 2*t*t + t;
  }

  h01(t){
    return -2*t*t*t + 3*t*t;
  }
  
  h11(t){
    return t*t*t - t*t;
  }

  get_position(t) {
    if(this.num_control_points < 2){
      return vec3(0, 0, 0);
    }

    if(t < 0){
      return this.points[0];
    }

    if(t >= 1){
      return this.points[this.num_control_points - 1];
    }
    
    // idx_a, and idx_b (Start and end indices of the segment)
    const num_segments = this.num_control_points - 1;
    const idx_a = Math.floor(t * num_segments);
    const idx_b = idx_a + 1;

    // local_t parameter within segment between idx_a and idx_b
    // local_t = (t - t_a) / (t_b - t_a) = (t * num_segments - idx_a/(num_segments)) / ((idx_b/(num_segments)) - (idx_a/(num_segments)))
    // = t * num_segments - idx_a
    const local_t = t * num_segments - idx_a;
    // Tangent scale t_{k+1} - t_k = (idx_b / num_segments) - (idx_a / num_segments) = 1 / num_segments
    const m_scale = 1 / num_segments;

    const p_a = this.points[idx_a];
    const m_a = this.tangents[idx_a];
    const p_b = this.points[idx_b];
    const m_b = this.tangents[idx_b];
    
    // p(x) = h00(t)p_k + h10(t)(x_{k+1} - x_k)m_k + h01(t)p_{k+1} + h11(t)(x_{k+1} - x_k)m_{k+1}
    return p_a.times(this.h00(local_t)).plus(m_a.times(this.h10(local_t) * m_scale)).plus(p_b.times(this.h01(local_t))).plus(m_b.times(this.h11(local_t) * m_scale));
  }
};

// Curve shape class

export class Curve_shape extends Shape{
  constructor( curve_func, num_samples, curve_color = color(1, 0, 0, 1) ){
    super( "position", "normal" );

    this.material = {shader: new defs.Phong_Shader(), color: curve_color, ambient: 1.0 };
    this.sample_count = num_samples;

    if (curve_func && this.sample_count) {
      for(let i = 0; i <= this.sample_count; i++) {
        const t = i / this.sample_count;
        this.arrays.position.push(curve_func(t));
        this.arrays.normal.push( vec3(0, 1, 0) ); // normal for Phong shader
      }
    }
  }

  draw(webgl_manager, uniforms){
    super.draw( webgl_manager, uniforms, Mat4.identity(), this.material, "LINE_STRIP" );
  }

  update(webgl_manager, uniforms, curve_func){
    if (curve_func && this.sample_count) {
      for(let i = 0; i <= this.sample_count; i++) {
        const t = i / this.sample_count;
        this.arrays.position[i] = curve_func(t);
      }
      this.copy_onto_graphics_card( webgl_manager.context );
    }
  }
};