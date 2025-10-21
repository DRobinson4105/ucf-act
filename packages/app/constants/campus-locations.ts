export interface CampusLocation {
  id: string;
  name: string;
  shortName: string;
  latitude: number;
  longitude: number;
  type: "academic" | "residential" | "dining" | "recreation" | "parking";
}

export const UCF_CENTER = {
  latitude: 28.6024,
  longitude: -81.2001,
};

export const CAMPUS_LOCATIONS: CampusLocation[] = [
  {
    id: "1",
    name: "John C. Hitt Library",
    shortName: "Library",
    latitude: 28.6013,
    longitude: -81.2003,
    type: "academic",
  },
  {
    id: "2",
    name: "Student Union",
    shortName: "Student Union",
    latitude: 28.6017,
    longitude: -81.2005,
    type: "dining",
  },
  {
    id: "3",
    name: "Engineering Building I",
    shortName: "Engineering I",
    latitude: 28.6021,
    longitude: -81.198,
    type: "academic",
  },
  {
    id: "4",
    name: "Nike Community",
    shortName: "Nike Dorms",
    latitude: 28.605,
    longitude: -81.201,
    type: "residential",
  },
  {
    id: "5",
    name: "Apollo Community",
    shortName: "Apollo Dorms",
    latitude: 28.5995,
    longitude: -81.202,
    type: "residential",
  },
  {
    id: "6",
    name: "Recreation and Wellness Center",
    shortName: "RWC",
    latitude: 28.6,
    longitude: -81.1975,
    type: "recreation",
  },
  {
    id: "7",
    name: "Mathematical Sciences Building",
    shortName: "Math Sciences",
    latitude: 28.603,
    longitude: -81.199,
    type: "academic",
  },
  {
    id: "8",
    name: "Garage A",
    shortName: "Parking A",
    latitude: 28.604,
    longitude: -81.2015,
    type: "parking",
  },
  {
    id: "9",
    name: "Visual Arts Building",
    shortName: "Visual Arts",
    latitude: 28.6008,
    longitude: -81.197,
    type: "academic",
  },
  {
    id: "10",
    name: "College of Medicine",
    shortName: "Medical",
    latitude: 28.605,
    longitude: -81.1985,
    type: "academic",
  },
];
