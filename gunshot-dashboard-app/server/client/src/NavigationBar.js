import React from "react";
import { Navbar, Container, Nav } from "react-bootstrap";

const NavigationBar = () => (
  <Navbar bg="dark" variant="dark">
    <Container>
    <Navbar.Brand href="/">Gunshot Detection</Navbar.Brand>
      <Nav className="me-auto">
        <Nav.Link href="/device-info">Devices</Nav.Link>
      </Nav>
    </Container>
  </Navbar>
);

export default NavigationBar;
