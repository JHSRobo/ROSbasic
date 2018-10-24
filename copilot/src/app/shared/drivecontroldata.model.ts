
export class DriveControlData {
    bools: [
        { name: String, value: boolean },
        { name: String, value: boolean },
        { name: String, value: boolean }
    ];
    doubles: [
        { name: String, value: number },
        { name: String, value: number },
        { name: String, value: number }
    ];
    groups: [
        { state: bool, name: string}
    ];
    ints: [
        { name: string, value: number },
        { name: string, value: number }
    ];
    strs: [
        length: 0;
    ]
}