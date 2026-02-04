export type RideStatus =
  | "pending"
  | "assigned"
  | "arriving"
  | "in-progress"
  | "completed"
  | "cancelled";

export interface Vehicle {
  id: string;
  model: string;
  color: string;
  licensePlate?: string;
}

export interface Ride {
  id: string;
  pickupLocationId: string;
  dropoffLocationId: string;
  status: RideStatus;
  requestedAt: Date;
  pickupAt?: Date;
  completedAt?: Date;
  estimatedWaitTime?: number;
  vehiclePosition?: { latitude: number; longitude: number };
  vehicle: Vehicle;
}

export interface Notification {
  id: string;
  title: string;
  message: string;
  timestamp: Date;
  read: boolean;
  type: "ride-update" | "system" | "promotion";
}
