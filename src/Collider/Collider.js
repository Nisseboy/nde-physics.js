class Collider extends Component {
  constructor() {
    super();

    this.r = undefined;

    this.physicsLayer = undefined;
    this.i = undefined;
  }

  from(data) {
    super.from(data);

    this.r = data.r;

    return this;
  }
}