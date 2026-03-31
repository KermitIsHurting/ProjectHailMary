#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "cuas_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__RadarDetection() -> *const std::ffi::c_void;
}

#[link(name = "cuas_msgs__rosidl_generator_c")]
extern "C" {
    fn cuas_msgs__msg__RadarDetection__init(msg: *mut RadarDetection) -> bool;
    fn cuas_msgs__msg__RadarDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RadarDetection>, size: usize) -> bool;
    fn cuas_msgs__msg__RadarDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RadarDetection>);
    fn cuas_msgs__msg__RadarDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RadarDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<RadarDetection>) -> bool;
}

// Corresponds to cuas_msgs__msg__RadarDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RadarDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for RadarDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cuas_msgs__msg__RadarDetection__init(&mut msg as *mut _) {
        panic!("Call to cuas_msgs__msg__RadarDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RadarDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__RadarDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__RadarDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__RadarDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RadarDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RadarDetection where Self: Sized {
  const TYPE_NAME: &'static str = "cuas_msgs/msg/RadarDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__RadarDetection() }
  }
}


#[link(name = "cuas_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__RadarFrame() -> *const std::ffi::c_void;
}

#[link(name = "cuas_msgs__rosidl_generator_c")]
extern "C" {
    fn cuas_msgs__msg__RadarFrame__init(msg: *mut RadarFrame) -> bool;
    fn cuas_msgs__msg__RadarFrame__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RadarFrame>, size: usize) -> bool;
    fn cuas_msgs__msg__RadarFrame__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RadarFrame>);
    fn cuas_msgs__msg__RadarFrame__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RadarFrame>, out_seq: *mut rosidl_runtime_rs::Sequence<RadarFrame>) -> bool;
}

// Corresponds to cuas_msgs__msg__RadarFrame
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RadarFrame {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for RadarFrame {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cuas_msgs__msg__RadarFrame__init(&mut msg as *mut _) {
        panic!("Call to cuas_msgs__msg__RadarFrame__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RadarFrame {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__RadarFrame__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__RadarFrame__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__RadarFrame__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RadarFrame {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RadarFrame where Self: Sized {
  const TYPE_NAME: &'static str = "cuas_msgs/msg/RadarFrame";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__RadarFrame() }
  }
}


#[link(name = "cuas_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__FusedDetection() -> *const std::ffi::c_void;
}

#[link(name = "cuas_msgs__rosidl_generator_c")]
extern "C" {
    fn cuas_msgs__msg__FusedDetection__init(msg: *mut FusedDetection) -> bool;
    fn cuas_msgs__msg__FusedDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FusedDetection>, size: usize) -> bool;
    fn cuas_msgs__msg__FusedDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FusedDetection>);
    fn cuas_msgs__msg__FusedDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FusedDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<FusedDetection>) -> bool;
}

// Corresponds to cuas_msgs__msg__FusedDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FusedDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for FusedDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cuas_msgs__msg__FusedDetection__init(&mut msg as *mut _) {
        panic!("Call to cuas_msgs__msg__FusedDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FusedDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__FusedDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__FusedDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__FusedDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FusedDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FusedDetection where Self: Sized {
  const TYPE_NAME: &'static str = "cuas_msgs/msg/FusedDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__FusedDetection() }
  }
}


#[link(name = "cuas_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__Track() -> *const std::ffi::c_void;
}

#[link(name = "cuas_msgs__rosidl_generator_c")]
extern "C" {
    fn cuas_msgs__msg__Track__init(msg: *mut Track) -> bool;
    fn cuas_msgs__msg__Track__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Track>, size: usize) -> bool;
    fn cuas_msgs__msg__Track__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Track>);
    fn cuas_msgs__msg__Track__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Track>, out_seq: *mut rosidl_runtime_rs::Sequence<Track>) -> bool;
}

// Corresponds to cuas_msgs__msg__Track
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Track {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for Track {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cuas_msgs__msg__Track__init(&mut msg as *mut _) {
        panic!("Call to cuas_msgs__msg__Track__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Track {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__Track__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__Track__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__Track__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Track {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Track where Self: Sized {
  const TYPE_NAME: &'static str = "cuas_msgs/msg/Track";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__Track() }
  }
}


#[link(name = "cuas_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__TrackArray() -> *const std::ffi::c_void;
}

#[link(name = "cuas_msgs__rosidl_generator_c")]
extern "C" {
    fn cuas_msgs__msg__TrackArray__init(msg: *mut TrackArray) -> bool;
    fn cuas_msgs__msg__TrackArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrackArray>, size: usize) -> bool;
    fn cuas_msgs__msg__TrackArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrackArray>);
    fn cuas_msgs__msg__TrackArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrackArray>, out_seq: *mut rosidl_runtime_rs::Sequence<TrackArray>) -> bool;
}

// Corresponds to cuas_msgs__msg__TrackArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for TrackArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cuas_msgs__msg__TrackArray__init(&mut msg as *mut _) {
        panic!("Call to cuas_msgs__msg__TrackArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrackArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__TrackArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__TrackArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__TrackArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrackArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrackArray where Self: Sized {
  const TYPE_NAME: &'static str = "cuas_msgs/msg/TrackArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__TrackArray() }
  }
}


#[link(name = "cuas_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__ThreatReport() -> *const std::ffi::c_void;
}

#[link(name = "cuas_msgs__rosidl_generator_c")]
extern "C" {
    fn cuas_msgs__msg__ThreatReport__init(msg: *mut ThreatReport) -> bool;
    fn cuas_msgs__msg__ThreatReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ThreatReport>, size: usize) -> bool;
    fn cuas_msgs__msg__ThreatReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ThreatReport>);
    fn cuas_msgs__msg__ThreatReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ThreatReport>, out_seq: *mut rosidl_runtime_rs::Sequence<ThreatReport>) -> bool;
}

// Corresponds to cuas_msgs__msg__ThreatReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ThreatReport {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ThreatReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cuas_msgs__msg__ThreatReport__init(&mut msg as *mut _) {
        panic!("Call to cuas_msgs__msg__ThreatReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ThreatReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__ThreatReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__ThreatReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cuas_msgs__msg__ThreatReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ThreatReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ThreatReport where Self: Sized {
  const TYPE_NAME: &'static str = "cuas_msgs/msg/ThreatReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cuas_msgs__msg__ThreatReport() }
  }
}


